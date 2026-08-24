#ifndef CHASSIS_SERVICE_H
#define CHASSIS_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    CHASSIS_SERVICE_STATUS_OK = 0,
    CHASSIS_SERVICE_STATUS_INVALID_PARAM,
    CHASSIS_SERVICE_STATUS_NOT_INITIALIZED,
    CHASSIS_SERVICE_STATUS_INIT_FAILED,
    CHASSIS_SERVICE_STATUS_DEVICE_ERROR,
    CHASSIS_SERVICE_STATUS_KINEMATICS_ERROR,
    CHASSIS_SERVICE_STATUS_FAULT_LATCHED,
    CHASSIS_SERVICE_STATUS_UNSAFE_STATE,
} ChassisServiceStatus;

typedef enum {
    CHASSIS_FAULT_NONE = 0,
    CHASSIS_FAULT_INITIALIZATION,
    CHASSIS_FAULT_DRIVE_TRANSMIT,
    CHASSIS_FAULT_STEER_TRANSMIT,
    CHASSIS_FAULT_DRIVE_RECEIVE_OVERFLOW,
    CHASSIS_FAULT_DRIVE_FEEDBACK_TIMEOUT,
    CHASSIS_FAULT_KINEMATICS,
    CHASSIS_FAULT_MANUAL_STOP,
} ChassisFault;

typedef struct {
    /** x/y 方向线速度，单位 m/s；z 轴角速度，单位 rad/s */
    float command_vx;
    float command_vy;
    float command_wz;
    uint32_t update_count;
    ChassisFault fault;
    bool initialized;
    bool remote_online;
    bool remote_enabled;
    bool fault_latched;
} ChassisServiceState;

ChassisServiceStatus chassis_service_init(void);
ChassisServiceStatus chassis_service_update(void);
ChassisServiceStatus chassis_service_stop(void);
/** 仅供人工确认安全后显式恢复；故障不会自动清除 */
ChassisServiceStatus chassis_service_fault_clear(void);
ChassisServiceStatus chassis_service_get_state(ChassisServiceState *out);
const char *chassis_service_status_str(ChassisServiceStatus status);
const char *chassis_service_fault_str(ChassisFault fault);

#endif /* CHASSIS_SERVICE_H */
