#include "wr_can_protocol.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

_Static_assert(sizeof(float) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24 &&
                   FLT_MAX_EXP == 128,
               "CAN protocol requires IEEE-754 float32");

typedef struct WrLayout {
    uint32_t id;
    uint8_t type;
    uint8_t logical_length;
    uint8_t wire_length;
} WrLayout;

static bool wr_layout_by_id(uint32_t id, WrLayout *layout)
{
    layout->id = id;
    switch (id) {
    case WR_CAN_ID_VELOCITY:     layout->type = WR_CAN_TYPE_VELOCITY; layout->logical_length = 28; layout->wire_length = 32; return true;
    case WR_CAN_ID_STEERING_TASK: layout->type = WR_CAN_TYPE_STEERING_TASK; layout->logical_length = 32; layout->wire_length = 32; return true;
    case WR_CAN_ID_CONTROL:      layout->type = WR_CAN_TYPE_CONTROL; layout->logical_length = 16; layout->wire_length = 16; return true;
    case WR_CAN_ID_TIME_REQUEST: layout->type = WR_CAN_TYPE_TIME_REQUEST; layout->logical_length = 16; layout->wire_length = 16; return true;
    case WR_CAN_ID_CHASSIS:      layout->type = WR_CAN_TYPE_CHASSIS; layout->logical_length = 52; layout->wire_length = 64; return true;
    case WR_CAN_ID_MODULE_FL:
    case WR_CAN_ID_MODULE_FR:
    case WR_CAN_ID_MODULE_RR:
    case WR_CAN_ID_MODULE_RL:    layout->type = WR_CAN_TYPE_MODULE; layout->logical_length = 64; layout->wire_length = 64; return true;
    case WR_CAN_ID_IMU:          layout->type = WR_CAN_TYPE_IMU; layout->logical_length = 48; layout->wire_length = 48; return true;
    case WR_CAN_ID_ORIENTATION:  layout->type = WR_CAN_TYPE_ORIENTATION; layout->logical_length = 36; layout->wire_length = 48; return true;
    case WR_CAN_ID_TASK_STATE:   layout->type = WR_CAN_TYPE_TASK_STATE; layout->logical_length = 20; layout->wire_length = 20; return true;
    case WR_CAN_ID_ACK:          layout->type = WR_CAN_TYPE_ACK; layout->logical_length = 17; layout->wire_length = 20; return true;
    case WR_CAN_ID_TIME_RESPONSE: layout->type = WR_CAN_TYPE_TIME_RESPONSE; layout->logical_length = 32; layout->wire_length = 32; return true;
    default: return false;
    }
}

static bool wr_layout_by_message(const WrCanMessage *message, WrLayout *layout)
{
    uint32_t id;
    switch (message->type) {
    case WR_CAN_TYPE_VELOCITY: id = WR_CAN_ID_VELOCITY; break;
    case WR_CAN_TYPE_STEERING_TASK: id = WR_CAN_ID_STEERING_TASK; break;
    case WR_CAN_TYPE_CONTROL: id = WR_CAN_ID_CONTROL; break;
    case WR_CAN_TYPE_TIME_REQUEST: id = WR_CAN_ID_TIME_REQUEST; break;
    case WR_CAN_TYPE_CHASSIS: id = WR_CAN_ID_CHASSIS; break;
    case WR_CAN_TYPE_MODULE:
        if (message->data.module.module > WR_MODULE_RL) return false;
        id = WR_CAN_ID_MODULE_FL + message->data.module.module;
        break;
    case WR_CAN_TYPE_IMU: id = WR_CAN_ID_IMU; break;
    case WR_CAN_TYPE_ORIENTATION: id = WR_CAN_ID_ORIENTATION; break;
    case WR_CAN_TYPE_TASK_STATE: id = WR_CAN_ID_TASK_STATE; break;
    case WR_CAN_TYPE_ACK: id = WR_CAN_ID_ACK; break;
    case WR_CAN_TYPE_TIME_RESPONSE: id = WR_CAN_ID_TIME_RESPONSE; break;
    default: return false;
    }
    return wr_layout_by_id(id, layout);
}

static void wr_put16(uint8_t *data, size_t offset, uint16_t value)
{
    data[offset] = (uint8_t)value;
    data[offset + 1] = (uint8_t)(value >> 8);
}

static void wr_put32(uint8_t *data, size_t offset, uint32_t value)
{
    for (size_t i = 0; i < 4; ++i) data[offset + i] = (uint8_t)(value >> (8u * i));
}

static void wr_put64(uint8_t *data, size_t offset, uint64_t value)
{
    for (size_t i = 0; i < 8; ++i) data[offset + i] = (uint8_t)(value >> (8u * i));
}

static void wr_put_float(uint8_t *data, size_t offset, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    wr_put32(data, offset, bits);
}

static uint16_t wr_get16(const uint8_t *data, size_t offset)
{
    return (uint16_t)((uint16_t)data[offset] | ((uint16_t)data[offset + 1] << 8));
}

static uint32_t wr_get32(const uint8_t *data, size_t offset)
{
    uint32_t value = 0;
    for (size_t i = 0; i < 4; ++i) value |= (uint32_t)data[offset + i] << (8u * i);
    return value;
}

static uint64_t wr_get64(const uint8_t *data, size_t offset)
{
    uint64_t value = 0;
    for (size_t i = 0; i < 8; ++i) value |= (uint64_t)data[offset + i] << (8u * i);
    return value;
}

static float wr_get_float(const uint8_t *data, size_t offset)
{
    uint32_t bits = wr_get32(data, offset);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static bool wr_valid_mask(uint8_t mask)
{
    return (mask & (uint8_t)~WR_CAN_ALL_MODULES) == 0;
}

static bool wr_valid_result(WrResult result)
{
    return (unsigned)result <= WR_RESULT_HARDWARE_FAULT;
}

static bool wr_valid_message(const WrCanMessage *message)
{
    switch (message->type) {
    case WR_CAN_TYPE_VELOCITY: {
        const WrVelocityCommand *v = &message->data.velocity;
        return isfinite(v->vx) && isfinite(v->vy) && isfinite(v->wz) &&
               v->valid_for_ms >= 1 && v->valid_for_ms <= WR_CAN_MAX_VELOCITY_VALID_MS &&
               v->session_id != 0;
    }
    case WR_CAN_TYPE_STEERING_TASK: {
        const WrSteeringTaskCommand *v = &message->data.steering_task;
        if (v->action > WR_ACTION_HOLD || !wr_valid_mask(v->module_mask) ||
            !isfinite(v->max_angle) || !isfinite(v->angular_speed) ||
            !isfinite(v->timeout_s) || v->session_id == 0) return false;
        return v->action != WR_ACTION_PROBE ||
               (v->module_mask != 0 && v->max_angle > 0 &&
                v->angular_speed > 0 && v->timeout_s > 0);
    }
    case WR_CAN_TYPE_CONTROL: {
        const WrControlCommand *v = &message->data.control;
        return v->source <= WR_SOURCE_AUTONOMOUS && v->mode <= WR_MODE_EXIT &&
               v->session_id != 0;
    }
    case WR_CAN_TYPE_TIME_REQUEST: return true;
    case WR_CAN_TYPE_CHASSIS: {
        const WrChassisState *v = &message->data.chassis;
        return v->source <= WR_SOURCE_AUTONOMOUS && v->mode <= WR_MODE_EXIT;
    }
    case WR_CAN_TYPE_MODULE: return message->data.module.module <= WR_MODULE_RL;
    case WR_CAN_TYPE_IMU: return (message->data.imu.status_bits & 0xf0u) == 0;
    case WR_CAN_TYPE_ORIENTATION: return (message->data.orientation.status_bits & 0xf0u) == 0;
    case WR_CAN_TYPE_TASK_STATE: {
        const WrSteeringTaskState *v = &message->data.task_state;
        return v->action <= WR_ACTION_HOLD && v->phase <= WR_TASK_FAULT &&
               wr_valid_mask(v->active_mask) && wr_valid_mask(v->holding_mask) &&
               wr_valid_mask(v->completed_mask) && wr_valid_mask(v->safety_stopped_mask) &&
               wr_valid_result(v->result);
    }
    case WR_CAN_TYPE_ACK: {
        const WrCommandAck *v = &message->data.ack;
        return (v->acknowledged_type == WR_CAN_TYPE_STEERING_TASK ||
                v->acknowledged_type == WR_CAN_TYPE_CONTROL) &&
               wr_valid_result(v->result) && wr_valid_mask(v->active_module_mask);
    }
    case WR_CAN_TYPE_TIME_RESPONSE: return true;
    default: return false;
    }
}

static bool wr_zero_range(const uint8_t *data, size_t begin, size_t end)
{
    for (size_t i = begin; i < end; ++i) if (data[i] != 0) return false;
    return true;
}

static bool wr_reserved_zero(const WrCanFrame *frame, const WrLayout *layout)
{
    const uint8_t *d = frame->data;
    if (!wr_zero_range(d, 2, 4) ||
        !wr_zero_range(d, layout->logical_length, frame->length)) return false;
    switch (layout->type) {
    case WR_CAN_TYPE_VELOCITY: return wr_zero_range(d, 22, 24);
    case WR_CAN_TYPE_STEERING_TASK: return wr_zero_range(d, 14, 16);
    case WR_CAN_TYPE_CONTROL: return d[11] == 0 && (d[10] & 0xf8u) == 0;
    case WR_CAN_TYPE_CHASSIS: return d[27] == 0 && (d[26] & 0xe0u) == 0;
    case WR_CAN_TYPE_MODULE: return wr_zero_range(d, 22, 24) && (d[17] & 0xe0u) == 0;
    default: return true;
    }
}

WrResult wr_can_decode(const WrCanFrame *frame, WrCanMessage *message)
{
    if (frame == NULL || message == NULL) return WR_RESULT_BAD_PARAMETER;

    WrLayout layout;
    if (!wr_layout_by_id(frame->id, &layout) || !frame->fd || !frame->brs ||
        frame->length != layout.wire_length || frame->data[0] != WR_CAN_VERSION ||
        frame->data[1] != layout.type || !wr_reserved_zero(frame, &layout))
        return WR_RESULT_BAD_FORMAT;

    const uint8_t *d = frame->data;
    WrCanMessage out = {0};
    out.type = layout.type;
    out.sequence = wr_get32(d, 4);
    switch (layout.type) {
    case WR_CAN_TYPE_VELOCITY: {
        WrVelocityCommand *v = &out.data.velocity;
        v->vx = wr_get_float(d, 8);
        v->vy = wr_get_float(d, 12);
        v->wz = wr_get_float(d, 16);
        v->valid_for_ms = wr_get16(d, 20);
        v->session_id = wr_get32(d, 24);
        break;
    }
    case WR_CAN_TYPE_STEERING_TASK: {
        WrSteeringTaskCommand *v = &out.data.steering_task;
        v->task_sequence = wr_get32(d, 8);
        v->action = d[12];
        v->module_mask = d[13];
        v->max_angle = wr_get_float(d, 16);
        v->angular_speed = wr_get_float(d, 20);
        v->timeout_s = wr_get_float(d, 24);
        v->session_id = wr_get32(d, 28);
        break;
    }
    case WR_CAN_TYPE_CONTROL: {
        WrControlCommand *v = &out.data.control;
        v->source = d[8];
        v->mode = d[9];
        v->enable = (d[10] & 0x01u) != 0;
        v->quick_stop = (d[10] & 0x02u) != 0;
        v->clear_fault = (d[10] & 0x04u) != 0;
        v->session_id = wr_get32(d, 12);
        break;
    }
    case WR_CAN_TYPE_TIME_REQUEST:
        out.data.time_request.host_send_time_ns = wr_get64(d, 8);
        break;
    case WR_CAN_TYPE_CHASSIS: {
        WrChassisState *v = &out.data.chassis;
        v->sample_time_us = wr_get64(d, 8);
        v->boot_id = wr_get32(d, 16);
        v->last_command_sequence = wr_get32(d, 20);
        v->source = d[24];
        v->mode = d[25];
        v->enabled = (d[26] & 0x01u) != 0;
        v->physical_estop = (d[26] & 0x02u) != 0;
        v->remote_online = (d[26] & 0x04u) != 0;
        v->command_timed_out = (d[26] & 0x08u) != 0;
        v->motor_power_ready = (d[26] & 0x10u) != 0;
        v->vx = wr_get_float(d, 28);
        v->vy = wr_get_float(d, 32);
        v->wz = wr_get_float(d, 36);
        v->fault_bits = wr_get32(d, 40);
        v->battery_voltage = wr_get_float(d, 44);
        v->battery_current = wr_get_float(d, 48);
        break;
    }
    case WR_CAN_TYPE_MODULE: {
        WrModuleState *v = &out.data.module;
        v->sample_time_us = wr_get64(d, 8);
        v->module = d[16];
        v->steering_online = (d[17] & 0x01u) != 0;
        v->drive_online = (d[17] & 0x02u) != 0;
        v->zero_valid = (d[17] & 0x04u) != 0;
        v->torque_valid = (d[17] & 0x08u) != 0;
        v->position_valid = (d[17] & 0x10u) != 0;
        v->steering_fault_bits = wr_get16(d, 18);
        v->drive_fault_bits = wr_get16(d, 20);
        v->steering_angle = wr_get_float(d, 24);
        v->steering_velocity = wr_get_float(d, 28);
        v->steering_current = wr_get_float(d, 32);
        v->steering_torque = wr_get_float(d, 36);
        v->drive_position = wr_get_float(d, 40);
        v->drive_velocity = wr_get_float(d, 44);
        v->drive_current = wr_get_float(d, 48);
        v->drive_torque = wr_get_float(d, 52);
        v->steering_temperature = wr_get_float(d, 56);
        v->drive_temperature = wr_get_float(d, 60);
        break;
    }
    case WR_CAN_TYPE_IMU: {
        WrImuSample *v = &out.data.imu;
        v->sample_time_us = wr_get64(d, 8);
        for (size_t i = 0; i < 3; ++i) {
            v->gyro[i] = wr_get_float(d, 16 + 4 * i);
            v->accel[i] = wr_get_float(d, 28 + 4 * i);
        }
        v->temperature = wr_get_float(d, 40);
        v->status_bits = wr_get32(d, 44);
        break;
    }
    case WR_CAN_TYPE_ORIENTATION: {
        WrImuOrientation *v = &out.data.orientation;
        v->sample_time_us = wr_get64(d, 8);
        for (size_t i = 0; i < 4; ++i) v->xyzw[i] = wr_get_float(d, 16 + 4 * i);
        v->status_bits = wr_get32(d, 32);
        break;
    }
    case WR_CAN_TYPE_TASK_STATE: {
        WrSteeringTaskState *v = &out.data.task_state;
        v->task_sequence = wr_get32(d, 8);
        v->action = d[12];
        v->phase = d[13];
        v->active_mask = d[14];
        v->holding_mask = d[15];
        v->completed_mask = d[16];
        v->safety_stopped_mask = d[17];
        v->result = (WrResult)wr_get16(d, 18);
        break;
    }
    case WR_CAN_TYPE_ACK: {
        WrCommandAck *v = &out.data.ack;
        v->acknowledged_sequence = wr_get32(d, 8);
        v->acknowledged_type = d[12];
        if (d[13] > 1) return WR_RESULT_BAD_PARAMETER;
        v->accepted = d[13] != 0;
        v->result = (WrResult)wr_get16(d, 14);
        v->active_module_mask = d[16];
        break;
    }
    case WR_CAN_TYPE_TIME_RESPONSE: {
        WrTimeResponse *v = &out.data.time_response;
        v->host_send_time_ns = wr_get64(d, 8);
        v->mcu_receive_time_us = wr_get64(d, 16);
        v->mcu_send_time_us = wr_get64(d, 24);
        break;
    }
    default: return WR_RESULT_BAD_FORMAT;
    }
    if (layout.type == WR_CAN_TYPE_MODULE &&
        out.data.module.module != frame->id - WR_CAN_ID_MODULE_FL)
        return WR_RESULT_BAD_PARAMETER;
    if (!wr_valid_message(&out)) return WR_RESULT_BAD_PARAMETER;
    *message = out;
    return WR_RESULT_SUCCESS;
}

WrResult wr_can_encode(const WrCanMessage *message, WrCanFrame *frame)
{
    if (message == NULL || frame == NULL) return WR_RESULT_BAD_PARAMETER;
    WrLayout layout;
    if (!wr_layout_by_message(message, &layout) || !wr_valid_message(message))
        return WR_RESULT_BAD_PARAMETER;

    WrCanFrame out = {0};
    out.id = layout.id;
    out.length = layout.wire_length;
    out.fd = true;
    out.brs = true;
    uint8_t *d = out.data;
    d[0] = WR_CAN_VERSION;
    d[1] = message->type;
    wr_put32(d, 4, message->sequence);
    switch (message->type) {
    case WR_CAN_TYPE_VELOCITY: {
        const WrVelocityCommand *v = &message->data.velocity;
        wr_put_float(d, 8, v->vx);
        wr_put_float(d, 12, v->vy);
        wr_put_float(d, 16, v->wz);
        wr_put16(d, 20, v->valid_for_ms);
        wr_put32(d, 24, v->session_id);
        break;
    }
    case WR_CAN_TYPE_STEERING_TASK: {
        const WrSteeringTaskCommand *v = &message->data.steering_task;
        wr_put32(d, 8, v->task_sequence);
        d[12] = v->action;
        d[13] = v->module_mask;
        wr_put_float(d, 16, v->max_angle);
        wr_put_float(d, 20, v->angular_speed);
        wr_put_float(d, 24, v->timeout_s);
        wr_put32(d, 28, v->session_id);
        break;
    }
    case WR_CAN_TYPE_CONTROL: {
        const WrControlCommand *v = &message->data.control;
        d[8] = v->source;
        d[9] = v->mode;
        d[10] = (uint8_t)v->enable | ((uint8_t)v->quick_stop << 1) |
                ((uint8_t)v->clear_fault << 2);
        wr_put32(d, 12, v->session_id);
        break;
    }
    case WR_CAN_TYPE_TIME_REQUEST:
        wr_put64(d, 8, message->data.time_request.host_send_time_ns);
        break;
    case WR_CAN_TYPE_CHASSIS: {
        const WrChassisState *v = &message->data.chassis;
        wr_put64(d, 8, v->sample_time_us);
        wr_put32(d, 16, v->boot_id);
        wr_put32(d, 20, v->last_command_sequence);
        d[24] = v->source;
        d[25] = v->mode;
        d[26] = (uint8_t)v->enabled | ((uint8_t)v->physical_estop << 1) |
                ((uint8_t)v->remote_online << 2) |
                ((uint8_t)v->command_timed_out << 3) |
                ((uint8_t)v->motor_power_ready << 4);
        wr_put_float(d, 28, v->vx);
        wr_put_float(d, 32, v->vy);
        wr_put_float(d, 36, v->wz);
        wr_put32(d, 40, v->fault_bits);
        wr_put_float(d, 44, v->battery_voltage);
        wr_put_float(d, 48, v->battery_current);
        break;
    }
    case WR_CAN_TYPE_MODULE: {
        const WrModuleState *v = &message->data.module;
        wr_put64(d, 8, v->sample_time_us);
        d[16] = v->module;
        d[17] = (uint8_t)v->steering_online | ((uint8_t)v->drive_online << 1) |
                ((uint8_t)v->zero_valid << 2) |
                ((uint8_t)v->torque_valid << 3) |
                ((uint8_t)v->position_valid << 4);
        wr_put16(d, 18, v->steering_fault_bits);
        wr_put16(d, 20, v->drive_fault_bits);
        wr_put_float(d, 24, v->steering_angle);
        wr_put_float(d, 28, v->steering_velocity);
        wr_put_float(d, 32, v->steering_current);
        wr_put_float(d, 36, v->steering_torque);
        wr_put_float(d, 40, v->drive_position);
        wr_put_float(d, 44, v->drive_velocity);
        wr_put_float(d, 48, v->drive_current);
        wr_put_float(d, 52, v->drive_torque);
        wr_put_float(d, 56, v->steering_temperature);
        wr_put_float(d, 60, v->drive_temperature);
        break;
    }
    case WR_CAN_TYPE_IMU: {
        const WrImuSample *v = &message->data.imu;
        wr_put64(d, 8, v->sample_time_us);
        for (size_t i = 0; i < 3; ++i) {
            wr_put_float(d, 16 + 4 * i, v->gyro[i]);
            wr_put_float(d, 28 + 4 * i, v->accel[i]);
        }
        wr_put_float(d, 40, v->temperature);
        wr_put32(d, 44, v->status_bits);
        break;
    }
    case WR_CAN_TYPE_ORIENTATION: {
        const WrImuOrientation *v = &message->data.orientation;
        wr_put64(d, 8, v->sample_time_us);
        for (size_t i = 0; i < 4; ++i) wr_put_float(d, 16 + 4 * i, v->xyzw[i]);
        wr_put32(d, 32, v->status_bits);
        break;
    }
    case WR_CAN_TYPE_TASK_STATE: {
        const WrSteeringTaskState *v = &message->data.task_state;
        wr_put32(d, 8, v->task_sequence);
        d[12] = v->action;
        d[13] = v->phase;
        d[14] = v->active_mask;
        d[15] = v->holding_mask;
        d[16] = v->completed_mask;
        d[17] = v->safety_stopped_mask;
        wr_put16(d, 18, (uint16_t)v->result);
        break;
    }
    case WR_CAN_TYPE_ACK: {
        const WrCommandAck *v = &message->data.ack;
        wr_put32(d, 8, v->acknowledged_sequence);
        d[12] = v->acknowledged_type;
        d[13] = (uint8_t)v->accepted;
        wr_put16(d, 14, (uint16_t)v->result);
        d[16] = v->active_module_mask;
        break;
    }
    case WR_CAN_TYPE_TIME_RESPONSE: {
        const WrTimeResponse *v = &message->data.time_response;
        wr_put64(d, 8, v->host_send_time_ns);
        wr_put64(d, 16, v->mcu_receive_time_us);
        wr_put64(d, 24, v->mcu_send_time_us);
        break;
    }
    default: return WR_RESULT_BAD_PARAMETER;
    }
    *frame = out;
    return WR_RESULT_SUCCESS;
}
