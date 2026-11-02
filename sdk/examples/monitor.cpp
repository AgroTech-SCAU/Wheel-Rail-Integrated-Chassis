#include "wheel_rail/client.hpp"

#include <chrono>
#include <csignal>
#include <iostream>

namespace {
volatile std::sig_atomic_t stop_requested = 0;

void on_signal(int) {
    stop_requested = 1;
}

void print_snapshot(const wheel_rail::Snapshot& state) {
    std::cout << "MCU 在线：" << (state.online ? "是" : "否")
              << "；当前会话：" << state.session_id << '\n';
    if (!state.chassis.fresh || !state.chassis.value) {
        std::cout << "整车状态：尚无新鲜样本\n";
        return;
    }

    const auto& chassis = *state.chassis.value;
    std::cout << "整车：boot_id=" << chassis.boot_id
              << "，控制源=" << static_cast<unsigned>(chassis.source)
              << "，模式=" << static_cast<unsigned>(chassis.mode)
              << "，使能=" << chassis.enabled
              << "，电机电源就绪=" << chassis.motor_power_ready
              << "，故障位=0x" << std::hex << chassis.fault_bits << std::dec
              << "，实测速度=(" << chassis.vx << ", " << chassis.vy
              << ", " << chassis.wz << ")\n";

    for (std::size_t i = 0; i < state.modules.size(); ++i) {
        const auto& module = state.modules[i];
        if (!module.fresh || !module.value) {
            std::cout << "模块 " << i << "：无新鲜样本\n";
            continue;
        }
        std::cout << "模块 " << i << "：转向角=" << module.value->steering_angle
                  << " rad，驱动速度=" << module.value->drive_velocity
                  << " rad/s，转矩有效=" << module.value->torque_valid << '\n';
    }

    if (state.imu.fresh && state.imu.value) {
        const auto& imu = *state.imu.value;
        std::cout << "IMU：角速度=(" << imu.gyro[0] << ", " << imu.gyro[1]
                  << ", " << imu.gyro[2] << ") rad/s，状态位=0x"
                  << std::hex << imu.status_bits << std::dec << '\n';
    } else {
        std::cout << "IMU：无新鲜样本\n";
    }
    std::cout << std::flush;
}
} // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "用法：" << argv[0] << " <SocketCAN接口名>\n"
                  << "示例：" << argv[0] << " vcan0\n";
        return 2;
    }

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    wheel_rail::Client client;
    if (const auto error = client.open(argv[1])) {
        std::cerr << "无法打开 SocketCAN 接口：" << error.message() << '\n';
        return 1;
    }

    auto next_print = wheel_rail::Clock::now();
    while (!stop_requested) {
        const auto result = client.poll(std::chrono::milliseconds{100});
        if (!result) {
            std::cerr << "接收失败：" << result.error.message() << '\n';
            return 1;
        }
        for (const auto& event : result.events) {
            if (event.kind == wheel_rail::EventKind::mcu_restarted) {
                std::cout << "检测到 MCU 重启；旧状态和时间同步已失效\n";
            } else if (event.kind == wheel_rail::EventKind::state_stale) {
                std::cout << "整车状态已过期\n";
            } else if (event.kind == wheel_rail::EventKind::link_error) {
                std::cout << "CAN 链路错误：" << event.error.message() << '\n';
            } else if (event.kind == wheel_rail::EventKind::protocol_error) {
                std::cout << "收到无效协议帧：" << event.error.message() << '\n';
            }
        }

        const auto now = wheel_rail::Clock::now();
        if (now >= next_print) {
            print_snapshot(client.snapshot());
            next_print = now + std::chrono::seconds{1};
        }
    }
    client.close();
    return 0;
}
