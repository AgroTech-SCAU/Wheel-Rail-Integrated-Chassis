#include "wheel_rail/protocol.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <type_traits>

namespace wheel_rail::protocol {
namespace {

static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
              "The wire protocol requires IEEE-754 float32");

constexpr std::uint32_t velocity_id = 0x100;
constexpr std::uint32_t task_command_id = 0x101;
constexpr std::uint32_t control_id = 0x102;
constexpr std::uint32_t sync_request_id = 0x103;
constexpr std::uint32_t chassis_id = 0x180;
constexpr std::uint32_t first_module_id = 0x181;
constexpr std::uint32_t last_module_id = 0x184;
constexpr std::uint32_t imu_id = 0x185;
constexpr std::uint32_t orientation_id = 0x186;
constexpr std::uint32_t task_state_id = 0x187;
constexpr std::uint32_t ack_id = 0x188;
constexpr std::uint32_t sync_response_id = 0x18b;

void put16(Frame& frame, std::size_t offset, std::uint16_t value) {
    frame.data[offset] = static_cast<std::uint8_t>(value);
    frame.data[offset + 1] = static_cast<std::uint8_t>(value >> 8);
}

void put32(Frame& frame, std::size_t offset, std::uint32_t value) {
    for(std::size_t i = 0; i < 4; ++i)
        frame.data[offset + i] = static_cast<std::uint8_t>(value >> (8 * i));
}

void put64(Frame& frame, std::size_t offset, std::uint64_t value) {
    for(std::size_t i = 0; i < 8; ++i)
        frame.data[offset + i] = static_cast<std::uint8_t>(value >> (8 * i));
}

void put_float(Frame& frame, std::size_t offset, float value) {
    std::uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    put32(frame, offset, bits);
}

std::uint16_t get16(const Frame& frame, std::size_t offset) {
    return static_cast<std::uint16_t>(frame.data[offset]) |
           static_cast<std::uint16_t>(frame.data[offset + 1] << 8);
}

std::uint32_t get32(const Frame& frame, std::size_t offset) {
    std::uint32_t value = 0;
    for(std::size_t i = 0; i < 4; ++i)
        value |= static_cast<std::uint32_t>(frame.data[offset + i]) << (8 * i);
    return value;
}

std::uint64_t get64(const Frame& frame, std::size_t offset) {
    std::uint64_t value = 0;
    for(std::size_t i = 0; i < 8; ++i)
        value |= static_cast<std::uint64_t>(frame.data[offset + i]) << (8 * i);
    return value;
}

float get_float(const Frame& frame, std::size_t offset) {
    const std::uint32_t bits = get32(frame, offset);
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

bool valid(ControlSource source) {
    return static_cast<std::uint8_t>(source) <= 2;
}

bool valid(Mode mode) {
    return static_cast<std::uint8_t>(mode) <= 3;
}

bool valid(Module module) {
    return static_cast<std::uint8_t>(module) <= 3;
}

bool valid(Action action) {
    return static_cast<std::uint8_t>(action) <= 4;
}

bool valid(TaskPhase phase) {
    return static_cast<std::uint8_t>(phase) <= 7;
}

bool valid(ResultCode result) {
    return static_cast<std::uint16_t>(result) <= 9;
}

bool valid_mask(std::uint8_t mask) {
    return (mask & static_cast<std::uint8_t>(~all_modules)) == 0;
}

bool valid_imu_status(std::uint32_t status) {
    return (status & 0x000000f0u) == 0;
}

Errc validate(const VelocityCommand& value) {
    if(!std::isfinite(value.vx) || !std::isfinite(value.vy) ||
       !std::isfinite(value.wz) || value.valid_for_ms == 0 ||
       value.valid_for_ms > maximum_velocity_validity_ms || value.session_id == 0)
        return Errc::invalid_value;
    return static_cast<Errc>(0);
}

Errc validate(const SteeringTaskCommand& value) {
    if(!valid(value.action) || !valid_mask(value.module_mask) ||
       !std::isfinite(value.max_angle) || !std::isfinite(value.angular_speed) ||
       !std::isfinite(value.timeout_s) || value.session_id == 0)
        return Errc::invalid_value;
    if(value.action == Action::probe &&
       (value.module_mask == 0 || value.max_angle <= 0 ||
        value.angular_speed <= 0 || value.timeout_s <= 0))
        return Errc::invalid_value;
    return static_cast<Errc>(0);
}

Errc validate(const ControlCommand& value) {
    if(!valid(value.source) || !valid(value.mode) || value.session_id == 0)
        return Errc::invalid_value;
    return static_cast<Errc>(0);
}

Errc validate(const TimeSyncRequest&) { return static_cast<Errc>(0); }

Errc validate(const ChassisState& value) {
    return valid(value.source) && valid(value.mode)
               ? static_cast<Errc>(0) : Errc::invalid_value;
}

Errc validate(const ModuleState& value) {
    return valid(value.module) ? static_cast<Errc>(0) : Errc::invalid_value;
}

Errc validate(const ImuSample& value) {
    return valid_imu_status(value.status_bits)
               ? static_cast<Errc>(0) : Errc::invalid_value;
}

Errc validate(const ImuOrientation& value) {
    return valid_imu_status(value.status_bits)
               ? static_cast<Errc>(0) : Errc::invalid_value;
}

Errc validate(const SteeringTaskState& value) {
    if(!valid(value.action) || !valid(value.phase) || !valid(value.result) ||
       !valid_mask(value.active_mask) || !valid_mask(value.holding_mask) ||
       !valid_mask(value.completed_mask) ||
       !valid_mask(value.safety_stopped_mask))
        return Errc::invalid_value;
    return static_cast<Errc>(0);
}

Errc validate(const CommandAck& value) {
    if((value.acknowledged_type != 0x11 && value.acknowledged_type != 0x12) ||
       !valid(value.result) || !valid_mask(value.active_module_mask))
        return Errc::invalid_value;
    return static_cast<Errc>(0);
}

Errc validate(const TimeSyncResponse&) { return static_cast<Errc>(0); }

template<class T>
void write_body(Frame& frame, const T& value) {
    if constexpr(std::is_same_v<T, VelocityCommand>) {
        put_float(frame, 8, value.vx);
        put_float(frame, 12, value.vy);
        put_float(frame, 16, value.wz);
        put16(frame, 20, value.valid_for_ms);
        put32(frame, 24, value.session_id);
    } else if constexpr(std::is_same_v<T, SteeringTaskCommand>) {
        put32(frame, 8, value.task_sequence);
        frame.data[12] = static_cast<std::uint8_t>(value.action);
        frame.data[13] = value.module_mask;
        put_float(frame, 16, value.max_angle);
        put_float(frame, 20, value.angular_speed);
        put_float(frame, 24, value.timeout_s);
        put32(frame, 28, value.session_id);
    } else if constexpr(std::is_same_v<T, ControlCommand>) {
        frame.data[8] = static_cast<std::uint8_t>(value.source);
        frame.data[9] = static_cast<std::uint8_t>(value.mode);
        frame.data[10] = static_cast<std::uint8_t>(value.enable) |
                         static_cast<std::uint8_t>(value.quick_stop) << 1 |
                         static_cast<std::uint8_t>(value.clear_fault) << 2;
        put32(frame, 12, value.session_id);
    } else if constexpr(std::is_same_v<T, TimeSyncRequest>) {
        put64(frame, 8, value.host_send_time_ns);
    } else if constexpr(std::is_same_v<T, ChassisState>) {
        put64(frame, 8, value.sample_time_us);
        put32(frame, 16, value.boot_id);
        put32(frame, 20, value.last_command_sequence);
        frame.data[24] = static_cast<std::uint8_t>(value.source);
        frame.data[25] = static_cast<std::uint8_t>(value.mode);
        frame.data[26] = static_cast<std::uint8_t>(value.enabled) |
                         static_cast<std::uint8_t>(value.physical_estop) << 1 |
                         static_cast<std::uint8_t>(value.remote_online) << 2 |
                         static_cast<std::uint8_t>(value.command_timed_out) << 3 |
                         static_cast<std::uint8_t>(value.motor_power_ready) << 4;
        put_float(frame, 28, value.vx);
        put_float(frame, 32, value.vy);
        put_float(frame, 36, value.wz);
        put32(frame, 40, value.fault_bits);
        put_float(frame, 44, value.battery_voltage);
        put_float(frame, 48, value.battery_current);
    } else if constexpr(std::is_same_v<T, ModuleState>) {
        put64(frame, 8, value.sample_time_us);
        frame.data[16] = static_cast<std::uint8_t>(value.module);
        frame.data[17] = static_cast<std::uint8_t>(value.steering_online) |
                         static_cast<std::uint8_t>(value.drive_online) << 1 |
                         static_cast<std::uint8_t>(value.zero_valid) << 2 |
                         static_cast<std::uint8_t>(value.torque_valid) << 3 |
                         static_cast<std::uint8_t>(value.position_valid) << 4;
        put16(frame, 18, value.steering_fault_bits);
        put16(frame, 20, value.drive_fault_bits);
        put_float(frame, 24, value.steering_angle);
        put_float(frame, 28, value.steering_velocity);
        put_float(frame, 32, value.steering_current);
        put_float(frame, 36, value.steering_torque);
        put_float(frame, 40, value.drive_position);
        put_float(frame, 44, value.drive_velocity);
        put_float(frame, 48, value.drive_current);
        put_float(frame, 52, value.drive_torque);
        put_float(frame, 56, value.steering_temperature);
        put_float(frame, 60, value.drive_temperature);
    } else if constexpr(std::is_same_v<T, ImuSample>) {
        put64(frame, 8, value.sample_time_us);
        for(std::size_t i = 0; i < 3; ++i) {
            put_float(frame, 16 + 4 * i, value.gyro[i]);
            put_float(frame, 28 + 4 * i, value.accel[i]);
        }
        put_float(frame, 40, value.temperature);
        put32(frame, 44, value.status_bits);
    } else if constexpr(std::is_same_v<T, ImuOrientation>) {
        put64(frame, 8, value.sample_time_us);
        for(std::size_t i = 0; i < 4; ++i)
            put_float(frame, 16 + 4 * i, value.xyzw[i]);
        put32(frame, 32, value.status_bits);
    } else if constexpr(std::is_same_v<T, SteeringTaskState>) {
        put32(frame, 8, value.task_sequence);
        frame.data[12] = static_cast<std::uint8_t>(value.action);
        frame.data[13] = static_cast<std::uint8_t>(value.phase);
        frame.data[14] = value.active_mask;
        frame.data[15] = value.holding_mask;
        frame.data[16] = value.completed_mask;
        frame.data[17] = value.safety_stopped_mask;
        put16(frame, 18, static_cast<std::uint16_t>(value.result));
    } else if constexpr(std::is_same_v<T, CommandAck>) {
        put32(frame, 8, value.acknowledged_sequence);
        frame.data[12] = value.acknowledged_type;
        frame.data[13] = static_cast<std::uint8_t>(value.accepted);
        put16(frame, 14, static_cast<std::uint16_t>(value.result));
        frame.data[16] = value.active_module_mask;
    } else if constexpr(std::is_same_v<T, TimeSyncResponse>) {
        put64(frame, 8, value.host_send_time_ns);
        put64(frame, 16, value.mcu_receive_time_us);
        put64(frame, 24, value.mcu_send_time_us);
    }
}

Errc check_reserved(const Frame& frame) {
    const auto zero = [&frame](std::size_t begin, std::size_t end) {
        for(std::size_t i = begin; i < end; ++i)
            if(frame.data[i] != 0) return false;
        return true;
    };
    if(!zero(2, 4) || !zero(logical_size(frame.id), frame.size))
        return Errc::nonzero_reserved;
    switch(frame.id) {
        case velocity_id: return zero(22, 24) ? static_cast<Errc>(0) : Errc::nonzero_reserved;
        case task_command_id: return zero(14, 16) ? static_cast<Errc>(0) : Errc::nonzero_reserved;
        case control_id: return zero(11, 12) && (frame.data[10] & 0xf8u) == 0
                                    ? static_cast<Errc>(0) : Errc::nonzero_reserved;
        case chassis_id: return frame.data[27] == 0 && (frame.data[26] & 0xe0u) == 0
                                    ? static_cast<Errc>(0) : Errc::nonzero_reserved;
        case 0x181: case 0x182: case 0x183: case 0x184:
            return zero(22, 24) && (frame.data[17] & 0xe0u) == 0
                       ? static_cast<Errc>(0) : Errc::nonzero_reserved;
        default: return static_cast<Errc>(0);
    }
}

Message read_body(const Frame& frame) {
    const Header header{get32(frame, 4)};
    switch(frame.id) {
        case velocity_id: {
            VelocityCommand value{};
            value.header = header;
            value.vx = get_float(frame, 8);
            value.vy = get_float(frame, 12);
            value.wz = get_float(frame, 16);
            value.valid_for_ms = get16(frame, 20);
            value.session_id = get32(frame, 24);
            return value;
        }
        case task_command_id: {
            SteeringTaskCommand value{};
            value.header = header;
            value.task_sequence = get32(frame, 8);
            value.action = static_cast<Action>(frame.data[12]);
            value.module_mask = frame.data[13];
            value.max_angle = get_float(frame, 16);
            value.angular_speed = get_float(frame, 20);
            value.timeout_s = get_float(frame, 24);
            value.session_id = get32(frame, 28);
            return value;
        }
        case control_id: {
            ControlCommand value{};
            value.header = header;
            value.source = static_cast<ControlSource>(frame.data[8]);
            value.mode = static_cast<Mode>(frame.data[9]);
            value.enable = (frame.data[10] & 0x01u) != 0;
            value.quick_stop = (frame.data[10] & 0x02u) != 0;
            value.clear_fault = (frame.data[10] & 0x04u) != 0;
            value.session_id = get32(frame, 12);
            return value;
        }
        case sync_request_id: return TimeSyncRequest{header, get64(frame, 8)};
        case chassis_id: {
            ChassisState value{};
            value.header = header;
            value.sample_time_us = get64(frame, 8);
            value.boot_id = get32(frame, 16);
            value.last_command_sequence = get32(frame, 20);
            value.source = static_cast<ControlSource>(frame.data[24]);
            value.mode = static_cast<Mode>(frame.data[25]);
            value.enabled = (frame.data[26] & 0x01u) != 0;
            value.physical_estop = (frame.data[26] & 0x02u) != 0;
            value.remote_online = (frame.data[26] & 0x04u) != 0;
            value.command_timed_out = (frame.data[26] & 0x08u) != 0;
            value.motor_power_ready = (frame.data[26] & 0x10u) != 0;
            value.vx = get_float(frame, 28);
            value.vy = get_float(frame, 32);
            value.wz = get_float(frame, 36);
            value.fault_bits = get32(frame, 40);
            value.battery_voltage = get_float(frame, 44);
            value.battery_current = get_float(frame, 48);
            return value;
        }
        case 0x181: case 0x182: case 0x183: case 0x184: {
            ModuleState value{};
            value.header = header;
            value.sample_time_us = get64(frame, 8);
            value.module = static_cast<Module>(frame.data[16]);
            value.steering_online = (frame.data[17] & 0x01u) != 0;
            value.drive_online = (frame.data[17] & 0x02u) != 0;
            value.zero_valid = (frame.data[17] & 0x04u) != 0;
            value.torque_valid = (frame.data[17] & 0x08u) != 0;
            value.position_valid = (frame.data[17] & 0x10u) != 0;
            value.steering_fault_bits = get16(frame, 18);
            value.drive_fault_bits = get16(frame, 20);
            value.steering_angle = get_float(frame, 24);
            value.steering_velocity = get_float(frame, 28);
            value.steering_current = get_float(frame, 32);
            value.steering_torque = get_float(frame, 36);
            value.drive_position = get_float(frame, 40);
            value.drive_velocity = get_float(frame, 44);
            value.drive_current = get_float(frame, 48);
            value.drive_torque = get_float(frame, 52);
            value.steering_temperature = get_float(frame, 56);
            value.drive_temperature = get_float(frame, 60);
            return value;
        }
        case imu_id: {
            ImuSample value{};
            value.header = header;
            value.sample_time_us = get64(frame, 8);
            for(std::size_t i = 0; i < 3; ++i) {
                value.gyro[i] = get_float(frame, 16 + 4 * i);
                value.accel[i] = get_float(frame, 28 + 4 * i);
            }
            value.temperature = get_float(frame, 40);
            value.status_bits = get32(frame, 44);
            return value;
        }
        case orientation_id: {
            ImuOrientation value{};
            value.header = header;
            value.sample_time_us = get64(frame, 8);
            for(std::size_t i = 0; i < 4; ++i)
                value.xyzw[i] = get_float(frame, 16 + 4 * i);
            value.status_bits = get32(frame, 32);
            return value;
        }
        case task_state_id: {
            SteeringTaskState value{};
            value.header = header;
            value.task_sequence = get32(frame, 8);
            value.action = static_cast<Action>(frame.data[12]);
            value.phase = static_cast<TaskPhase>(frame.data[13]);
            value.active_mask = frame.data[14];
            value.holding_mask = frame.data[15];
            value.completed_mask = frame.data[16];
            value.safety_stopped_mask = frame.data[17];
            value.result = static_cast<ResultCode>(get16(frame, 18));
            return value;
        }
        case ack_id: {
            CommandAck value{};
            value.header = header;
            value.acknowledged_sequence = get32(frame, 8);
            value.acknowledged_type = frame.data[12];
            value.accepted = frame.data[13] != 0;
            value.result = static_cast<ResultCode>(get16(frame, 14));
            value.active_module_mask = frame.data[16];
            return value;
        }
        default: return TimeSyncResponse{header, get64(frame, 8), get64(frame, 16),
                                         get64(frame, 24)};
    }
}

} // namespace

std::uint32_t can_id(const Message& message) noexcept {
    return std::visit([](const auto& value) -> std::uint32_t {
        using T = std::decay_t<decltype(value)>;
        if constexpr(std::is_same_v<T, VelocityCommand>) return velocity_id;
        else if constexpr(std::is_same_v<T, SteeringTaskCommand>) return task_command_id;
        else if constexpr(std::is_same_v<T, ControlCommand>) return control_id;
        else if constexpr(std::is_same_v<T, TimeSyncRequest>) return sync_request_id;
        else if constexpr(std::is_same_v<T, ChassisState>) return chassis_id;
        else if constexpr(std::is_same_v<T, ModuleState>)
            return first_module_id + static_cast<std::uint8_t>(value.module);
        else if constexpr(std::is_same_v<T, ImuSample>) return imu_id;
        else if constexpr(std::is_same_v<T, ImuOrientation>) return orientation_id;
        else if constexpr(std::is_same_v<T, SteeringTaskState>) return task_state_id;
        else if constexpr(std::is_same_v<T, CommandAck>) return ack_id;
        else return sync_response_id;
    }, message);
}

std::uint8_t message_type(const Message& message) noexcept {
    return std::visit([](const auto& value) -> std::uint8_t {
        using T = std::decay_t<decltype(value)>;
        if constexpr(std::is_same_v<T, VelocityCommand>) return 0x10;
        else if constexpr(std::is_same_v<T, SteeringTaskCommand>) return 0x11;
        else if constexpr(std::is_same_v<T, ControlCommand>) return 0x12;
        else if constexpr(std::is_same_v<T, TimeSyncRequest>) return 0x13;
        else if constexpr(std::is_same_v<T, ChassisState>) return 0x20;
        else if constexpr(std::is_same_v<T, ModuleState>) return 0x21;
        else if constexpr(std::is_same_v<T, ImuSample>) return 0x22;
        else if constexpr(std::is_same_v<T, ImuOrientation>) return 0x23;
        else if constexpr(std::is_same_v<T, SteeringTaskState>) return 0x24;
        else if constexpr(std::is_same_v<T, CommandAck>) return 0x25;
        else return 0x28;
    }, message);
}

std::uint8_t logical_size(std::uint32_t id) noexcept {
    switch(id) {
        case velocity_id: return 28;
        case task_command_id: return 32;
        case control_id: return 16;
        case sync_request_id: return 16;
        case chassis_id: return 52;
        case 0x181: case 0x182: case 0x183: case 0x184: return 64;
        case imu_id: return 48;
        case orientation_id: return 36;
        case task_state_id: return 20;
        case ack_id: return 17;
        case sync_response_id: return 32;
        default: return 0;
    }
}

std::uint8_t wire_size(std::uint32_t id) noexcept {
    switch(id) {
        case velocity_id: return 32;
        case chassis_id: return 64;
        case orientation_id: return 48;
        case ack_id: return 20;
        default: return logical_size(id);
    }
}

EncodeResult encode(const Message& message) {
    const Errc value_error = std::visit([](const auto& value) { return validate(value); }, message);
    if(static_cast<int>(value_error) != 0)
        return {{}, make_error_code(value_error)};

    Frame frame{};
    frame.id = can_id(message);
    frame.size = wire_size(frame.id);
    if(frame.size == 0)
        return {{}, make_error_code(Errc::invalid_value)};
    frame.data[0] = wire_version;
    frame.data[1] = message_type(message);
    std::visit([&frame](const auto& value) {
        put32(frame, 4, value.header.sequence);
        write_body(frame, value);
    }, message);
    return {frame, {}};
}

DecodeResult decode(const Frame& frame) {
    if(logical_size(frame.id) == 0)
        return {{}, make_error_code(Errc::unsupported_id)};
    if(!frame.fd || !frame.bitrate_switch)
        return {{}, make_error_code(Errc::invalid_frame)};
    if(frame.size != wire_size(frame.id))
        return {{}, make_error_code(Errc::invalid_length)};
    if(frame.data[0] != wire_version)
        return {{}, make_error_code(Errc::unsupported_version)};

    const Message message = read_body(frame);
    if(frame.data[1] != message_type(message))
        return {{}, make_error_code(Errc::invalid_type)};
    const Errc reserved_error = check_reserved(frame);
    if(static_cast<int>(reserved_error) != 0)
        return {{}, make_error_code(reserved_error)};
    if(frame.id >= first_module_id && frame.id <= last_module_id &&
       static_cast<std::uint8_t>(std::get<ModuleState>(message).module) !=
           frame.id - first_module_id)
        return {{}, make_error_code(Errc::invalid_value)};
    if(frame.id == ack_id && frame.data[13] > 1)
        return {{}, make_error_code(Errc::invalid_value)};
    const Errc value_error = std::visit([](const auto& value) { return validate(value); }, message);
    if(static_cast<int>(value_error) != 0)
        return {{}, make_error_code(value_error)};
    return {message, {}};
}

} // namespace wheel_rail::protocol
