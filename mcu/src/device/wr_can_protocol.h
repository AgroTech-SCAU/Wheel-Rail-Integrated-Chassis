#ifndef WR_CAN_PROTOCOL_H
#define WR_CAN_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    WR_CAN_VERSION = 1,
    WR_CAN_MAX_VELOCITY_VALID_MS = 200,
    WR_CAN_ALL_MODULES = 0x0f,
    WR_CAN_FRONT_MODULES = 0x03,
    WR_CAN_ID_VELOCITY = 0x100,
    WR_CAN_ID_STEERING_TASK = 0x101,
    WR_CAN_ID_CONTROL = 0x102,
    WR_CAN_ID_TIME_REQUEST = 0x103,
    WR_CAN_ID_CHASSIS = 0x180,
    WR_CAN_ID_MODULE_FL = 0x181,
    WR_CAN_ID_MODULE_FR = 0x182,
    WR_CAN_ID_MODULE_RR = 0x183,
    WR_CAN_ID_MODULE_RL = 0x184,
    WR_CAN_ID_IMU = 0x185,
    WR_CAN_ID_ORIENTATION = 0x186,
    WR_CAN_ID_TASK_STATE = 0x187,
    WR_CAN_ID_ACK = 0x188,
    WR_CAN_ID_TIME_RESPONSE = 0x18b,
    WR_CAN_TYPE_VELOCITY = 0x10,
    WR_CAN_TYPE_STEERING_TASK = 0x11,
    WR_CAN_TYPE_CONTROL = 0x12,
    WR_CAN_TYPE_TIME_REQUEST = 0x13,
    WR_CAN_TYPE_CHASSIS = 0x20,
    WR_CAN_TYPE_MODULE = 0x21,
    WR_CAN_TYPE_IMU = 0x22,
    WR_CAN_TYPE_ORIENTATION = 0x23,
    WR_CAN_TYPE_TASK_STATE = 0x24,
    WR_CAN_TYPE_ACK = 0x25,
    WR_CAN_TYPE_TIME_RESPONSE = 0x28
};

typedef enum WrResult {
    WR_RESULT_SUCCESS = 0,
    WR_RESULT_BAD_FORMAT = 1,
    WR_RESULT_BAD_PARAMETER = 2,
    WR_RESULT_MODE_NOT_ALLOWED = 3,
    WR_RESULT_UNAUTHORIZED = 4,
    WR_RESULT_BUSY = 5,
    WR_RESULT_STALE_SEQUENCE = 6,
    WR_RESULT_TIMEOUT = 7,
    WR_RESULT_SAFETY_STOP = 8,
    WR_RESULT_HARDWARE_FAULT = 9
} WrResult;

enum {
    WR_SOURCE_NONE = 0, WR_SOURCE_MANUAL = 1, WR_SOURCE_AUTONOMOUS = 2,
    WR_MODE_FREE = 0, WR_MODE_ENTRY = 1, WR_MODE_ON_RAIL = 2, WR_MODE_EXIT = 3,
    WR_ACTION_IDLE = 0, WR_ACTION_PROBE = 1, WR_ACTION_CANCEL = 2,
    WR_ACTION_RELEASE = 3, WR_ACTION_HOLD = 4,
    WR_TASK_IDLE = 0, WR_TASK_EXECUTING = 1, WR_TASK_HOLDING = 2,
    WR_TASK_COMPLETED = 3, WR_TASK_CANCELLED = 4, WR_TASK_TIMEOUT = 5,
    WR_TASK_SAFETY_STOPPED = 6, WR_TASK_FAULT = 7,
    WR_MODULE_FL = 0, WR_MODULE_FR = 1, WR_MODULE_RR = 2, WR_MODULE_RL = 3
};

typedef struct WrCanFrame {
    /* The transport must reject extended-ID frames before passing their numeric ID here. */
    uint32_t id;
    uint8_t length;
    bool fd;
    bool brs;
    uint8_t data[64];
} WrCanFrame;

typedef struct WrVelocityCommand {
    float vx, vy, wz;
    uint16_t valid_for_ms;
    uint32_t session_id;
} WrVelocityCommand;

typedef struct WrControlCommand {
    uint8_t source, mode;
    bool enable, quick_stop, clear_fault;
    uint32_t session_id;
} WrControlCommand;

typedef struct WrSteeringTaskCommand {
    uint32_t task_sequence;
    uint8_t action, module_mask;
    float max_angle, angular_speed, timeout_s;
    uint32_t session_id;
} WrSteeringTaskCommand;

typedef struct WrTimeRequest {
    uint64_t host_send_time_ns;
} WrTimeRequest;

typedef struct WrChassisState {
    uint64_t sample_time_us;
    uint32_t boot_id, last_command_sequence;
    uint8_t source, mode;
    bool enabled, physical_estop, remote_online, command_timed_out, motor_power_ready;
    float vx, vy, wz;
    uint32_t fault_bits;
    float battery_voltage, battery_current;
} WrChassisState;

typedef struct WrModuleState {
    uint64_t sample_time_us;
    uint8_t module;
    bool steering_online, drive_online, zero_valid, torque_valid, position_valid;
    uint16_t steering_fault_bits, drive_fault_bits;
    float steering_angle, steering_velocity, steering_current, steering_torque;
    float drive_position, drive_velocity, drive_current, drive_torque;
    float steering_temperature, drive_temperature;
} WrModuleState;

typedef struct WrImuSample {
    uint64_t sample_time_us;
    float gyro[3], accel[3];
    float temperature;
    uint32_t status_bits;
} WrImuSample;

typedef struct WrImuOrientation {
    uint64_t sample_time_us;
    float xyzw[4];
    uint32_t status_bits;
} WrImuOrientation;

typedef struct WrSteeringTaskState {
    uint32_t task_sequence;
    uint8_t action, phase;
    uint8_t active_mask, holding_mask, completed_mask, safety_stopped_mask;
    WrResult result;
} WrSteeringTaskState;

typedef struct WrCommandAck {
    uint32_t acknowledged_sequence;
    uint8_t acknowledged_type;
    bool accepted;
    WrResult result;
    uint8_t active_module_mask;
} WrCommandAck;

typedef struct WrTimeResponse {
    uint64_t host_send_time_ns, mcu_receive_time_us, mcu_send_time_us;
} WrTimeResponse;

typedef struct WrCanMessage {
    uint8_t type;
    uint32_t sequence;
    union {
        WrVelocityCommand velocity;
        WrControlCommand control;
        WrSteeringTaskCommand steering_task;
        WrTimeRequest time_request;
        WrChassisState chassis;
        WrModuleState module;
        WrImuSample imu;
        WrImuOrientation orientation;
        WrSteeringTaskState task_state;
        WrCommandAck ack;
        WrTimeResponse time_response;
    } data;
} WrCanMessage;

WrResult wr_can_decode(const WrCanFrame *frame, WrCanMessage *message);
WrResult wr_can_encode(const WrCanMessage *message, WrCanFrame *frame);

#ifdef __cplusplus
}
#endif

#endif
