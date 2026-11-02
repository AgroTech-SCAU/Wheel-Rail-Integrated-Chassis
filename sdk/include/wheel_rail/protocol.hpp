#pragma once

#include "wheel_rail/error.hpp"
#include <array>
#include <cstdint>
#include <variant>

namespace wheel_rail::protocol {
constexpr std::uint8_t wire_version = 1;
constexpr std::uint16_t maximum_velocity_validity_ms = 200;

struct Frame {
    std::uint32_t id{};
    std::uint8_t size{};
    bool fd{true};
    bool bitrate_switch{true};
    std::array<std::uint8_t, 64> data{};
};
struct Header { std::uint32_t sequence{}; };
enum class ControlSource : std::uint8_t { none = 0, manual = 1, autonomous = 2 };
enum class Mode : std::uint8_t { free_surface = 0, entry = 1, on_rail = 2, exit = 3 };
enum class Module : std::uint8_t { fl = 0, fr = 1, rr = 2, rl = 3 };
enum class Action : std::uint8_t { idle = 0, probe = 1, cancel = 2, release = 3, hold = 4 };
enum class TaskPhase : std::uint8_t {
    idle = 0, executing = 1, holding = 2, completed = 3,
    cancelled = 4, timeout = 5, safety_stopped = 6, fault = 7
};
enum class ResultCode : std::uint16_t {
    success = 0, bad_format = 1, bad_parameter = 2, mode_not_allowed = 3,
    unauthorized = 4, busy = 5, stale_sequence = 6, timeout = 7,
    safety_stop = 8, hardware_fault = 9
};
constexpr std::uint8_t all_modules = 0x0f;
constexpr std::uint8_t front_modules = 0x03;

struct VelocityCommand {
    Header header;
    float vx{}, vy{}, wz{};
    std::uint16_t valid_for_ms{200};
    std::uint32_t session_id{};
};
struct SteeringTaskCommand {
    Header header;
    std::uint32_t task_sequence{};
    Action action{Action::idle};
    std::uint8_t module_mask{};
    float max_angle{}, angular_speed{}, timeout_s{};
    std::uint32_t session_id{};
};
struct ControlCommand {
    Header header;
    ControlSource source{ControlSource::none};
    Mode mode{Mode::free_surface};
    bool enable{}, quick_stop{}, clear_fault{};
    std::uint32_t session_id{};
};
struct TimeSyncRequest { Header header; std::uint64_t host_send_time_ns{}; };

struct ChassisState {
    Header header;
    std::uint64_t sample_time_us{};
    std::uint32_t boot_id{}, last_command_sequence{};
    ControlSource source{ControlSource::none};
    Mode mode{Mode::free_surface};
    bool enabled{}, physical_estop{}, remote_online{}, command_timed_out{}, motor_power_ready{};
    float vx{}, vy{}, wz{};
    std::uint32_t fault_bits{};
    float battery_voltage{}, battery_current{};
};
struct ModuleState {
    Header header;
    std::uint64_t sample_time_us{};
    Module module{Module::fl};
    bool steering_online{}, drive_online{}, zero_valid{}, torque_valid{}, position_valid{};
    std::uint16_t steering_fault_bits{}, drive_fault_bits{};
    float steering_angle{}, steering_velocity{}, steering_current{}, steering_torque{};
    float drive_position{}, drive_velocity{}, drive_current{}, drive_torque{};
    float steering_temperature{}, drive_temperature{};
};
struct ImuSample {
    Header header;
    std::uint64_t sample_time_us{};
    std::array<float, 3> gyro{}, accel{};
    float temperature{};
    std::uint32_t status_bits{};
};
struct ImuOrientation {
    Header header;
    std::uint64_t sample_time_us{};
    std::array<float, 4> xyzw{};
    std::uint32_t status_bits{};
};
struct SteeringTaskState {
    Header header;
    std::uint32_t task_sequence{};
    Action action{Action::idle};
    TaskPhase phase{TaskPhase::idle};
    std::uint8_t active_mask{}, holding_mask{}, completed_mask{}, safety_stopped_mask{};
    ResultCode result{ResultCode::success};
};
struct CommandAck {
    Header header;
    std::uint32_t acknowledged_sequence{};
    std::uint8_t acknowledged_type{};
    bool accepted{};
    ResultCode result{ResultCode::success};
    std::uint8_t active_module_mask{};
};
struct TimeSyncResponse {
    Header header;
    std::uint64_t host_send_time_ns{}, mcu_receive_time_us{}, mcu_send_time_us{};
};
using Message = std::variant<VelocityCommand, SteeringTaskCommand, ControlCommand,
    TimeSyncRequest, ChassisState, ModuleState, ImuSample, ImuOrientation,
    SteeringTaskState, CommandAck, TimeSyncResponse>;
struct EncodeResult { Frame frame; std::error_code error; explicit operator bool() const noexcept { return !error; } };
struct DecodeResult { Message message; std::error_code error; explicit operator bool() const noexcept { return !error; } };

// Encoders emit canonical CAN-FD DLC lengths with zero tail padding.
EncodeResult encode(const Message& message);
DecodeResult decode(const Frame& frame);
std::uint32_t can_id(const Message& message) noexcept;
std::uint8_t message_type(const Message& message) noexcept;
std::uint8_t wire_size(std::uint32_t can_id) noexcept;
std::uint8_t logical_size(std::uint32_t can_id) noexcept;
} // namespace wheel_rail::protocol
