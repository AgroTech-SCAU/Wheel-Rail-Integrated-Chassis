#pragma once

#include "wheel_rail/transport.hpp"
#include <array>
#include <chrono>
#include <cstddef>
#include <memory>
#include <optional>
#include <vector>

namespace wheel_rail {
using Clock = std::chrono::steady_clock;

struct ClientConfig {
    // Zero generates a new nonzero session at each open; fixed IDs are caller-owned.
    std::uint32_t session_id{};
    std::chrono::milliseconds state_timeout{100};
    std::chrono::milliseconds module_timeout{100};
    std::chrono::milliseconds imu_timeout{100};
    std::chrono::milliseconds ack_timeout{200};
    std::chrono::milliseconds time_sync_timeout{5000};
    std::size_t pending_limit{32};
};
struct Ticket {
    std::uint32_t sequence{};
    std::uint8_t type{};
    std::uint32_t session_id{};
};
struct SendResult {
    Ticket ticket;
    std::error_code error;
    explicit operator bool() const noexcept { return !error; }
};
template<class T> struct Sample {
    std::optional<T> value;
    Clock::time_point received_at{};
    bool fresh{}; // Evaluated at snapshot() time, not updated in returned copies.
};
struct TimeSyncEstimate {
    std::int64_t host_minus_mcu_ns{};
    std::chrono::nanoseconds network_round_trip{};
    Clock::time_point updated_at{};
    std::uint32_t boot_id{};
};
struct Snapshot {
    bool online{}, authorized{};
    std::uint32_t session_id{};
    Sample<protocol::ChassisState> chassis;
    std::array<Sample<protocol::ModuleState>, 4> modules;
    Sample<protocol::ImuSample> imu;
    Sample<protocol::ImuOrientation> orientation;
    Sample<protocol::SteeringTaskState> task;
    std::optional<TimeSyncEstimate> time_sync;
};
enum class EventKind {
    message, acknowledgement, acknowledgement_timeout, protocol_error,
    link_error, mcu_restarted, authority_lost, state_stale, command_cancelled,
    time_synchronized
};
struct Event {
    EventKind kind{EventKind::message};
    std::optional<protocol::Message> message;
    std::optional<Ticket> ticket;
    std::error_code error;
    std::uint32_t can_error_bits{};
};
struct PollResult {
    std::vector<Event> events;
    std::size_t received_frames{};
    std::error_code error;
    explicit operator bool() const noexcept { return !error; }
};

// All methods are called serially by one owner thread. No worker or auto-resend.
class Client {
public:
    explicit Client(ClientConfig config = {});
    Client(std::unique_ptr<Transport> transport, ClientConfig config = {});
    ~Client();
    Client(const Client&) = delete;
    Client& operator=(const Client&) = delete;

    std::error_code open(const std::string& interface_name);
    void close() noexcept;
    bool is_open() const noexcept;
    std::uint32_t session_id() const noexcept;
    PollResult poll(std::chrono::milliseconds timeout = std::chrono::milliseconds{0},
                    std::size_t max_frames = 64);
    Snapshot snapshot() const;

    SendResult send_velocity(float vx, float vy, float wz, std::uint16_t valid_for_ms = 200);
    SendResult request_autonomy(protocol::Mode mode, bool enable = true);
    SendResult release_control();
    SendResult steering_task(std::uint32_t task_sequence, protocol::Action action,
                             std::uint8_t module_mask, float max_angle = 0,
                             float angular_speed = 0, float timeout_s = 0);
    SendResult quick_stop();
    SendResult clear_fault();
    SendResult request_time_sync();
    std::optional<Clock::time_point> to_host_time(std::uint64_t mcu_time_us) const;

private:
    enum class Intent { other, autonomy, release, stop, clear };
    struct Pending {
        Ticket ticket;
        Clock::time_point sent_at;
        Intent intent;
        protocol::Mode mode;
        bool enable;
        std::optional<std::uint32_t> boot_id;
    };
    struct SyncPending {
        std::uint64_t host_send_ns;
        Clock::time_point sent_at;
        std::uint32_t boot_id;
    };
    SendResult send(protocol::Message message, bool acknowledgement, Intent intent = Intent::other,
                    protocol::Mode mode = protocol::Mode::free_surface, bool enable = false);
    void handle(const protocol::Message& message, Clock::time_point received_at, std::vector<Event>& events);
    void maintain(Clock::time_point now, std::vector<Event>& events);
    void invalidate(std::error_code reason, std::vector<Event>& events);
    bool authorized(Clock::time_point now) const;
    bool state_fresh(Clock::time_point now) const;
    protocol::Mode known_mode() const;

    std::unique_ptr<Transport> transport_;
    ClientConfig config_;
    Snapshot state_;
    std::vector<Pending> pending_;
    std::vector<SyncPending> sync_pending_;
    std::vector<Event> queued_events_;
    std::optional<std::uint32_t> boot_id_;
    std::array<std::optional<std::uint32_t>, 12> received_sequences_{};
    std::uint32_t session_id_{}, next_sequence_{};
    bool autonomy_acked_{}, online_seen_{}, authority_seen_{}, link_valid_{};
    protocol::Mode granted_mode_{protocol::Mode::free_surface};
    Clock::time_point grant_requested_at_{};
};
} // namespace wheel_rail
