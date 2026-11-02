#include "wheel_rail/client.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <linux/can/error.h>
#include <random>
#include <type_traits>

namespace wheel_rail {
namespace {
std::uint32_t new_id() {
    std::random_device random;
    const auto value = static_cast<std::uint32_t>(random());
    return value == 0 ? 1 : value;
}
template<class T> bool fresh(const Sample<T>& sample, Clock::time_point now,
                             std::chrono::milliseconds age) {
    return sample.value && now >= sample.received_at && now - sample.received_at < age;
}
template<class T> void assign(Sample<T>& sample, const T& value, Clock::time_point now) {
    sample.value = value;
    sample.received_at = now;
    sample.fresh = true;
}
Event failure(EventKind kind, std::error_code error, std::optional<Ticket> ticket = {}) {
    Event event;
    event.kind = kind;
    event.error = error;
    event.ticket = ticket;
    return event;
}
bool valid_config(const ClientConfig& config) {
    return config.state_timeout.count() > 0 && config.module_timeout.count() > 0 &&
           config.imu_timeout.count() > 0 && config.ack_timeout.count() > 0 &&
           config.time_sync_timeout.count() > 0 && config.pending_limit > 0;
}
} // namespace

Client::Client(ClientConfig config)
    : Client(std::make_unique<SocketCanTransport>(), config) {}

Client::Client(std::unique_ptr<Transport> transport, ClientConfig config)
    : transport_(std::move(transport)), config_(config) {}

Client::~Client() { close(); }

std::error_code Client::open(const std::string& interface_name) {
    if (!transport_ || !valid_config(config_)) return Errc::invalid_value;
    if (is_open()) return std::make_error_code(std::errc::already_connected);
    // Prepare identities before opening a resource; random_device can throw.
    const auto session = config_.session_id ? config_.session_id : new_id();
    const auto sequence = new_id();
    const auto error = transport_->open(interface_name);
    if (error) return error;
    state_ = {};
    pending_.clear();
    sync_pending_.clear();
    queued_events_.clear();
    boot_id_.reset();
    received_sequences_ = {};
    session_id_ = session;
    next_sequence_ = sequence;
    autonomy_acked_ = online_seen_ = authority_seen_ = link_valid_ = false;
    state_.session_id = session_id_;
    return {};
}

void Client::close() noexcept {
    if (transport_) transport_->close();
    state_ = {};
    pending_.clear();
    sync_pending_.clear();
    queued_events_.clear();
    boot_id_.reset();
    received_sequences_ = {};
    session_id_ = 0;
    autonomy_acked_ = online_seen_ = authority_seen_ = link_valid_ = false;
}

bool Client::is_open() const noexcept { return transport_ && transport_->is_open(); }
std::uint32_t Client::session_id() const noexcept { return session_id_; }

bool Client::state_fresh(Clock::time_point now) const {
    return is_open() && link_valid_ && fresh(state_.chassis, now, config_.state_timeout);
}

bool Client::authorized(Clock::time_point now) const {
    if (!autonomy_acked_ || !state_fresh(now)) return false;
    const auto& chassis = *state_.chassis.value;
    return state_.chassis.received_at >= grant_requested_at_ &&
           chassis.source == protocol::ControlSource::autonomous &&
           chassis.mode == granted_mode_ && chassis.enabled && chassis.motor_power_ready &&
           !chassis.physical_estop && chassis.fault_bits == 0;
}

protocol::Mode Client::known_mode() const {
    return state_.chassis.value ? state_.chassis.value->mode : protocol::Mode::free_surface;
}

Snapshot Client::snapshot() const {
    auto result = state_;
    const auto now = Clock::now();
    result.online = state_fresh(now);
    result.authorized = authorized(now);
    result.chassis.fresh = result.online;
    for (auto& module : result.modules) module.fresh = result.online && fresh(module, now, config_.module_timeout);
    result.imu.fresh = result.online && fresh(result.imu, now, config_.imu_timeout);
    result.orientation.fresh = result.online && fresh(result.orientation, now, config_.imu_timeout);
    result.task.fresh = result.online && fresh(result.task, now, config_.state_timeout);
    if (!result.online || (result.time_sync && now - result.time_sync->updated_at >= config_.time_sync_timeout))
        result.time_sync.reset();
    return result;
}

void Client::invalidate(std::error_code reason, std::vector<Event>& events) {
    if (autonomy_acked_ || authority_seen_) events.push_back(failure(EventKind::authority_lost, reason));
    autonomy_acked_ = authority_seen_ = false;
    for (const auto& pending : pending_)
        events.push_back(failure(EventKind::command_cancelled, reason, pending.ticket));
    pending_.clear();
    sync_pending_.clear();
    state_.time_sync.reset();
}

void Client::maintain(Clock::time_point now, std::vector<Event>& events) {
    if (online_seen_ && !state_fresh(now)) {
        online_seen_ = false;
        link_valid_ = false;
        invalidate(Errc::stale_state, events);
        events.push_back(failure(EventKind::state_stale, Errc::stale_state));
    }
    for (auto it = pending_.begin(); it != pending_.end();) {
        if (now - it->sent_at >= config_.ack_timeout) {
            events.push_back(failure(EventKind::acknowledgement_timeout, Errc::ack_timeout, it->ticket));
            it = pending_.erase(it);
        } else ++it;
    }
    sync_pending_.erase(std::remove_if(sync_pending_.begin(), sync_pending_.end(), [&](const SyncPending& pending) {
        return now - pending.sent_at >= config_.ack_timeout;
    }), sync_pending_.end());
}

SendResult Client::send(protocol::Message message, bool acknowledgement, Intent intent,
                        protocol::Mode mode, bool enable) {
    if (!is_open()) return {{}, Errc::not_open};
    maintain(Clock::now(), queued_events_);
    if (acknowledgement && pending_.size() >= config_.pending_limit)
        return {{}, Errc::pending_limit};
    const Ticket ticket{next_sequence_++, protocol::message_type(message), session_id_};
    std::visit([&](auto& value) { value.header.sequence = ticket.sequence; }, message);
    const auto encoded = protocol::encode(message);
    if (!encoded) return {ticket, encoded.error};
    const auto now = Clock::now();
    const auto error = transport_->send(encoded.frame);
    if (error) return {ticket, error};
    if (acknowledgement) pending_.push_back({ticket, now, intent, mode, enable, boot_id_});
    if (intent == Intent::autonomy) {
        autonomy_acked_ = authority_seen_ = false;
        grant_requested_at_ = now;
        granted_mode_ = mode;
    }
    return {ticket, {}};
}

SendResult Client::send_velocity(float vx, float vy, float wz, std::uint16_t valid_for_ms) {
    if (!is_open()) return {{}, Errc::not_open};
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz) ||
        valid_for_ms == 0 || valid_for_ms > protocol::maximum_velocity_validity_ms)
        return {{}, Errc::invalid_value};
    if (vx != 0 || vy != 0 || wz != 0) {
        if (!authorized(Clock::now())) return {{}, Errc::not_authorized};
        if (known_mode() == protocol::Mode::on_rail && (vy != 0 || wz != 0))
            return {{}, Errc::invalid_value};
    }
    protocol::VelocityCommand command;
    command.vx = vx; command.vy = vy; command.wz = wz;
    command.valid_for_ms = valid_for_ms;
    command.session_id = session_id_;
    return send(command, false);
}

SendResult Client::request_autonomy(protocol::Mode mode, bool enable) {
    if (!is_open()) return {{}, Errc::not_open};
    maintain(Clock::now(), queued_events_);
    if (!state_fresh(Clock::now())) return {{}, Errc::not_ready};
    if (std::any_of(pending_.begin(), pending_.end(), [](const Pending& value) { return value.ticket.type == 0x12; }))
        return {{}, std::make_error_code(std::errc::operation_in_progress)};
    protocol::ControlCommand command;
    command.source = protocol::ControlSource::autonomous;
    command.mode = mode; command.enable = enable; command.session_id = session_id_;
    return send(command, true, Intent::autonomy, mode, enable);
}

SendResult Client::release_control() {
    if (!is_open()) return {{}, Errc::not_open};
    invalidate(Errc::command_cancelled, queued_events_);
    protocol::ControlCommand command;
    command.mode = known_mode(); command.session_id = session_id_;
    return send(command, true, Intent::release);
}

SendResult Client::quick_stop() {
    if (!is_open()) return {{}, Errc::not_open};
    invalidate(Errc::command_cancelled, queued_events_);
    protocol::ControlCommand command;
    command.quick_stop = true; command.mode = known_mode(); command.session_id = session_id_;
    return send(command, true, Intent::stop);
}

SendResult Client::clear_fault() {
    if (!is_open()) return {{}, Errc::not_open};
    invalidate(Errc::command_cancelled, queued_events_);
    protocol::ControlCommand command;
    command.clear_fault = true; command.mode = known_mode(); command.session_id = session_id_;
    return send(command, true, Intent::clear);
}

SendResult Client::steering_task(std::uint32_t task_sequence, protocol::Action action,
                                 std::uint8_t module_mask, float max_angle,
                                 float angular_speed, float timeout_s) {
    if (!is_open()) return {{}, Errc::not_open};
    if (action != protocol::Action::cancel && !authorized(Clock::now()))
        return {{}, Errc::not_authorized};
    protocol::SteeringTaskCommand command;
    command.task_sequence = task_sequence; command.action = action;
    command.module_mask = module_mask; command.max_angle = max_angle;
    command.angular_speed = angular_speed; command.timeout_s = timeout_s;
    command.session_id = session_id_;
    return send(command, true);
}

SendResult Client::request_time_sync() {
    if (!is_open()) return {{}, Errc::not_open};
    maintain(Clock::now(), queued_events_);
    if (!state_fresh(Clock::now()) || !boot_id_) return {{}, Errc::not_ready};
    if (sync_pending_.size() >= config_.pending_limit) return {{}, Errc::pending_limit};
    const auto now = Clock::now();
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    if (ns < 0) return {{}, Errc::invalid_value};
    protocol::TimeSyncRequest request;
    request.host_send_time_ns = static_cast<std::uint64_t>(ns);
    const auto result = send(request, false);
    if (result) sync_pending_.push_back({request.host_send_time_ns, now, *boot_id_});
    return result;
}

void Client::handle(const protocol::Message& message, Clock::time_point received_at, std::vector<Event>& events) {
    const auto* chassis = std::get_if<protocol::ChassisState>(&message);
    if (chassis && (!boot_id_ || *boot_id_ != chassis->boot_id)) received_sequences_ = {};
    const auto index = static_cast<std::size_t>(protocol::can_id(message) - 0x180);
    const auto sequence = std::visit([](const auto& value) { return value.header.sequence; }, message);
    auto& previous = received_sequences_[index];
    if (previous) {
        const auto delta = sequence - *previous;
        if (delta == 0 || delta >= 0x80000000u) {
            events.push_back(failure(EventKind::protocol_error, Errc::stale_sequence));
            return;
        }
    }
    previous = sequence;
    Event event;
    event.message = message;
    std::visit([&](const auto& value) {
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T, protocol::ChassisState>) {
            if (!boot_id_ || *boot_id_ != value.boot_id) {
                const bool restarted = boot_id_.has_value();
                if (restarted) invalidate(Errc::mcu_restarted, events);
                state_ = {};
                state_.session_id = session_id_;
                boot_id_ = value.boot_id;
                if (restarted) events.push_back(failure(EventKind::mcu_restarted, Errc::mcu_restarted));
            }
            assign(state_.chassis, value, received_at);
            online_seen_ = true;
            link_valid_ = true;
            const bool awaiting_autonomy = std::any_of(pending_.begin(), pending_.end(), [](const Pending& pending) {
                return pending.intent == Intent::autonomy;
            });
            if ((autonomy_acked_ || awaiting_autonomy) && (value.source == protocol::ControlSource::manual ||
                value.physical_estop || value.fault_bits != 0 ||
                (authority_seen_ && (value.source != protocol::ControlSource::autonomous || !value.enabled || value.mode != granted_mode_))))
                invalidate(Errc::not_authorized, events);
            if (authorized(received_at)) authority_seen_ = true;
        } else if constexpr (std::is_same_v<T, protocol::ModuleState>) {
            if (boot_id_) assign(state_.modules[static_cast<std::size_t>(value.module)], value, received_at);
        } else if constexpr (std::is_same_v<T, protocol::ImuSample>) {
            if (boot_id_) assign(state_.imu, value, received_at);
        } else if constexpr (std::is_same_v<T, protocol::ImuOrientation>) {
            if (boot_id_) assign(state_.orientation, value, received_at);
        } else if constexpr (std::is_same_v<T, protocol::SteeringTaskState>) {
            if (boot_id_) assign(state_.task, value, received_at);
        } else if constexpr (std::is_same_v<T, protocol::CommandAck>) {
            const auto found = std::find_if(pending_.begin(), pending_.end(), [&](const Pending& pending) {
                return pending.ticket.sequence == value.acknowledged_sequence && pending.ticket.type == value.acknowledged_type &&
                       (!pending.boot_id || pending.boot_id == boot_id_);
            });
            if (found != pending_.end()) {
                event.kind = EventKind::acknowledgement;
                event.ticket = found->ticket;
                if (found->intent == Intent::autonomy) {
                    autonomy_acked_ = value.accepted && value.result == protocol::ResultCode::success && found->enable;
                    grant_requested_at_ = found->sent_at;
                    granted_mode_ = found->mode;
                    authority_seen_ = authorized(received_at);
                }
                pending_.erase(found);
            }
        } else if constexpr (std::is_same_v<T, protocol::TimeSyncResponse>) {
            const auto found = std::find_if(sync_pending_.begin(), sync_pending_.end(), [&](const SyncPending& pending) {
                return pending.host_send_ns == value.host_send_time_ns && boot_id_ && pending.boot_id == *boot_id_;
            });
            if (found != sync_pending_.end() && state_fresh(received_at)) {
                const auto receive_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(received_at.time_since_epoch()).count();
                constexpr auto max_us = static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max() / 1000);
                if (receive_ns >= 0 && value.host_send_time_ns <= static_cast<std::uint64_t>(receive_ns) &&
                    value.mcu_receive_time_us <= value.mcu_send_time_us && value.mcu_send_time_us <= max_us) {
                    const auto t1 = static_cast<std::int64_t>(value.host_send_time_ns);
                    const auto t2 = static_cast<std::int64_t>(value.mcu_receive_time_us * 1000);
                    const auto t3 = static_cast<std::int64_t>(value.mcu_send_time_us * 1000);
                    const auto rtt = (receive_ns - t1) - (t3 - t2);
                    if (rtt >= 0) {
                        state_.time_sync = TimeSyncEstimate{(t1 + (receive_ns - t1) / 2) - (t2 + (t3 - t2) / 2),
                            std::chrono::nanoseconds{rtt}, received_at, *boot_id_};
                        event.kind = EventKind::time_synchronized;
                    } else event = failure(EventKind::protocol_error, Errc::invalid_value);
                } else event = failure(EventKind::protocol_error, Errc::invalid_value);
                sync_pending_.erase(found);
            }
        }
    }, message);
    events.push_back(std::move(event));
}

PollResult Client::poll(std::chrono::milliseconds timeout, std::size_t max_frames) {
    PollResult result;
    result.events.swap(queued_events_);
    if (!is_open()) { result.error = Errc::not_open; return result; }
    if (timeout.count() < 0 || max_frames == 0) { result.error = Errc::invalid_value; return result; }
    maintain(Clock::now(), result.events);
    auto wait = timeout;
    const auto now = Clock::now();
    for (const auto& pending : pending_) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(pending.sent_at + config_.ack_timeout - now);
        wait = std::min(wait, std::max(std::chrono::milliseconds{0}, remaining));
    }
    if (online_seen_) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(state_.chassis.received_at + config_.state_timeout - now);
        wait = std::min(wait, std::max(std::chrono::milliseconds{0}, remaining));
    }
    for (std::size_t i = 0; i < max_frames; ++i) {
        protocol::Frame frame;
        const auto received = transport_->receive(frame, i == 0 ? wait : std::chrono::milliseconds{0});
        if (received.status == ReceiveStatus::timeout) break;
        if (received.status == ReceiveStatus::error) {
            result.error = received.error;
            link_valid_ = online_seen_ = false;
            invalidate(received.error, result.events);
            result.events.push_back(failure(EventKind::link_error, received.error));
            break;
        }
        if (received.status == ReceiveStatus::can_error) {
            auto event = failure(EventKind::link_error, received.error);
            event.can_error_bits = received.can_error_bits;
            result.events.push_back(std::move(event));
            if (received.can_error_bits & (CAN_ERR_BUSOFF | CAN_ERR_RESTARTED)) {
                link_valid_ = online_seen_ = false;
                invalidate(Errc::not_authorized, result.events);
            }
            continue;
        }
        ++result.received_frames;
        const auto decoded = protocol::decode(frame);
        const auto time = Clock::now();
        maintain(time, result.events);
        if (!decoded) result.events.push_back(failure(EventKind::protocol_error, decoded.error));
        else if (protocol::can_id(decoded.message) < 0x180)
            result.events.push_back(failure(EventKind::protocol_error, Errc::unsupported_id));
        else handle(decoded.message, time, result.events);
    }
    maintain(Clock::now(), result.events);
    return result;
}

std::optional<Clock::time_point> Client::to_host_time(std::uint64_t mcu_time_us) const {
    const auto current = snapshot();
    constexpr auto max_us = static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max() / 1000);
    if (!current.time_sync || mcu_time_us > max_us) return {};
    const auto value = static_cast<std::int64_t>(mcu_time_us * 1000);
    const auto offset = current.time_sync->host_minus_mcu_ns;
    if ((offset > 0 && value > std::numeric_limits<std::int64_t>::max() - offset) ||
        (offset < 0 && value < -offset)) return {};
    return Clock::time_point{std::chrono::duration_cast<Clock::duration>(std::chrono::nanoseconds{value + offset})};
}
} // namespace wheel_rail
