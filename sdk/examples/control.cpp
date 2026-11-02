#include "wheel_rail/client.hpp"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>
#include <variant>

namespace {
volatile std::sig_atomic_t stop_requested = 0;

void on_signal(int) {
    stop_requested = 1;
}

std::optional<float> parse_float(const char* text) {
    char* end = nullptr;
    errno = 0;
    const float value = std::strtof(text, &end);
    if (errno != 0 || end == text || *end != '\0' || !std::isfinite(value)) {
        return std::nullopt;
    }
    return value;
}

std::optional<wheel_rail::protocol::Mode> parse_mode(const std::string& text) {
    using wheel_rail::protocol::Mode;
    if (text == "free") return Mode::free_surface;
    if (text == "entry") return Mode::entry;
    if (text == "rail") return Mode::on_rail;
    if (text == "exit") return Mode::exit;
    return std::nullopt;
}

void print_usage(const char* program) {
    std::cerr << "用法：" << program
              << " <SocketCAN接口名> request-autonomy <free|entry|rail|exit>"
                 " <vx_m/s> <vy_m/s> <wz_rad/s> <持续秒数>\n"
              << "所有运动参数和持续时间均须显式提供；不带参数运行不会发控制帧\n";
}

bool same_ticket(const wheel_rail::Event& event, const wheel_rail::Ticket& ticket) {
    return event.ticket && event.ticket->sequence == ticket.sequence &&
           event.ticket->type == ticket.type &&
           event.ticket->session_id == ticket.session_id;
}

bool ready_for_motion(const wheel_rail::Snapshot& state,
                      wheel_rail::protocol::Mode requested_mode) {
    if (!state.online || !state.authorized || !state.chassis.fresh ||
        !state.chassis.value) {
        return false;
    }
    const auto& chassis = *state.chassis.value;
    return chassis.source == wheel_rail::protocol::ControlSource::autonomous &&
           chassis.mode == requested_mode && chassis.enabled &&
           chassis.motor_power_ready && !chassis.physical_estop &&
           chassis.fault_bits == 0;
}

bool stop_and_release(wheel_rail::Client& client) {
    const auto stop = client.quick_stop();
    if (stop) {
        std::cout << "软件快速停车请求已交给本机网络栈；需以 MCU ACK 和整车状态确认执行\n";
    } else {
        std::cerr << "软件快速停车请求发送失败：" << stop.error.message() << '\n';
    }
    (void)client.poll(std::chrono::milliseconds{250});

    const auto release = client.release_control();
    if (release) {
        std::cout << "释放控制权请求已交给本机网络栈\n";
    } else {
        std::cerr << "释放控制权请求发送失败：" << release.error.message() << '\n';
    }
    (void)client.poll(std::chrono::milliseconds{250});
    return static_cast<bool>(stop) && static_cast<bool>(release);
}
} // namespace

int main(int argc, char** argv) {
    if (argc != 8 || std::string(argv[2]) != "request-autonomy") {
        print_usage(argv[0]);
        return 2;
    }
    const auto mode = parse_mode(argv[3]);
    const auto vx = parse_float(argv[4]);
    const auto vy = parse_float(argv[5]);
    const auto wz = parse_float(argv[6]);
    const auto duration_s = parse_float(argv[7]);
    if (!mode || !vx || !vy || !wz || !duration_s ||
        *duration_s <= 0.0f || *duration_s > 60.0f ||
        (*mode == wheel_rail::protocol::Mode::on_rail &&
         (*vy != 0.0f || *wz != 0.0f))) {
        std::cerr << "参数无效；轨内模式要求 vy 和 wz 为零，持续时间须在 0～60 秒内\n";
        return 2;
    }

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    wheel_rail::Client client;
    if (const auto error = client.open(argv[1])) {
        std::cerr << "无法打开 SocketCAN 接口：" << error.message() << '\n';
        return 1;
    }

    const auto state_deadline = wheel_rail::Clock::now() + std::chrono::seconds{5};
    while (!stop_requested && wheel_rail::Clock::now() < state_deadline) {
        const auto received = client.poll(std::chrono::milliseconds{20});
        if (!received) {
            std::cerr << "接收失败：" << received.error.message() << '\n';
            return 1;
        }
        const auto state = client.snapshot();
        if (state.online && state.chassis.fresh && state.chassis.value) {
            if (state.chassis.value->physical_estop ||
                state.chassis.value->fault_bits != 0) {
                std::cerr << "MCU 报告急停或故障；未请求控制权\n";
                return 1;
            }
            break;
        }
    }
    if (stop_requested) return 0;
    const auto initial_state = client.snapshot();
    if (!initial_state.online || !initial_state.chassis.fresh ||
        !initial_state.chassis.value) {
        std::cerr << "未收到新鲜的 MCU 整车状态；未发送控制请求\n";
        return 1;
    }

    const auto request = client.request_autonomy(*mode, true);
    if (!request) {
        std::cerr << "控制请求未能在本机发送：" << request.error.message() << '\n';
        return 1;
    }
    std::cout << "控制请求已在本机发送；等待 MCU ACK 和实际状态\n";

    bool accepted = false;
    bool cannot_proceed = false;
    const auto grant_deadline = wheel_rail::Clock::now() + std::chrono::seconds{3};
    while (!stop_requested && wheel_rail::Clock::now() < grant_deadline) {
        const auto received = client.poll(std::chrono::milliseconds{20});
        if (!received) {
            std::cerr << "接收失败：" << received.error.message() << '\n';
            break;
        }
        for (const auto& event : received.events) {
            if (!same_ticket(event, request.ticket)) continue;
            if (event.kind == wheel_rail::EventKind::acknowledgement) {
                const auto* ack = event.message
                    ? std::get_if<wheel_rail::protocol::CommandAck>(&*event.message)
                    : nullptr;
                if (ack && ack->accepted) {
                    accepted = true;
                    std::cout << "MCU 已接受控制请求；仍等待整车状态确认生效\n";
                } else {
                    cannot_proceed = true;
                    std::cerr << "MCU 拒绝控制请求"
                              << (ack ? "，结果码=" + std::to_string(static_cast<unsigned>(ack->result))
                                      : "")
                              << '\n';
                }
            } else if (event.kind == wheel_rail::EventKind::acknowledgement_timeout) {
                std::cerr << "控制请求 ACK 超时；MCU 是否执行未知\n";
                cannot_proceed = true;
            } else if (event.kind == wheel_rail::EventKind::command_cancelled ||
                       event.kind == wheel_rail::EventKind::mcu_restarted) {
                std::cerr << "控制请求因状态变化失效\n";
                cannot_proceed = true;
            }
        }
        if (cannot_proceed || (accepted && ready_for_motion(client.snapshot(), *mode))) {
            break;
        }
    }

    bool completed_motion = false;
    if (!stop_requested && accepted && !cannot_proceed &&
        ready_for_motion(client.snapshot(), *mode)) {
        std::cout << "ACK 与实际控制状态均已确认；开始按 50 Hz 发送新速度帧\n";
        const auto duration = std::chrono::duration_cast<wheel_rail::Clock::duration>(
            std::chrono::duration<double>{*duration_s});
        const auto deadline = wheel_rail::Clock::now() + duration;
        auto next_send = wheel_rail::Clock::now();
        bool interrupted = false;
        while (!stop_requested && wheel_rail::Clock::now() < deadline) {
            const auto state = client.snapshot();
            if (!ready_for_motion(state, *mode)) {
                std::cerr << "控制权、使能或整车状态失效；停止发送运动速度\n";
                interrupted = true;
                break;
            }
            const auto now = wheel_rail::Clock::now();
            if (now >= next_send) {
                const auto sent = client.send_velocity(*vx, *vy, *wz, 200);
                if (!sent) {
                    std::cerr << "速度帧未能在本机发送：" << sent.error.message() << '\n';
                    interrupted = true;
                    break;
                }
                next_send = now + std::chrono::milliseconds{20};
            }
            const auto received = client.poll(std::chrono::milliseconds{5});
            if (!received) {
                std::cerr << "接收失败：" << received.error.message() << '\n';
                interrupted = true;
                break;
            }
        }
        completed_motion = !interrupted && !stop_requested &&
                           wheel_rail::Clock::now() >= deadline;
    } else if (!cannot_proceed && !stop_requested) {
        std::cerr << "未同时取得 ACK 与实际使能状态；未发送运动速度\n";
    }

    const bool stop_requests_sent = stop_and_release(client);
    client.close();
    return completed_motion && stop_requests_sent ? 0 : 1;
}
