#ifndef BENMO_DRIVE_MOTOR_H
#define BENMO_DRIVE_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#define BENMO_DRIVE_MOTOR_COUNT                4U
#define BENMO_DRIVE_MOTOR_ID_MIN               1U
#define BENMO_DRIVE_MOTOR_ID_MAX               4U
#define BENMO_DRIVE_MOTOR_SPEED_COMMAND_CAN_ID 0x032U
#define BENMO_DRIVE_MOTOR_QUERY_CAN_ID         0x107U
#define BENMO_DRIVE_MOTOR_CAN_DATA_LEN         8U
#define BENMO_DRIVE_MOTOR_RPM_MIN              (-210)
#define BENMO_DRIVE_MOTOR_RPM_MAX              210
#define BENMO_DRIVE_MOTOR_RAMP_STEP_RPM        5

/** 四节点本末驱动电机设备；速度单位统一为 RPM */

typedef enum {
    BENMO_DRIVE_STATUS_OK = 0,
    BENMO_DRIVE_STATUS_INVALID_PARAM,
    BENMO_DRIVE_STATUS_NOT_INITIALIZED,
    BENMO_DRIVE_STATUS_PORT_ERROR,
    BENMO_DRIVE_STATUS_FEEDBACK_TIMEOUT,
} BenmoDriveMotorStatus;

typedef struct {
    bool (*send_standard)(uint32_t id, const uint8_t *data, uint8_t len);
    uint32_t (*now_ms)(void);
    uint32_t (*critical_enter)(void);
    void (*critical_exit)(uint32_t state);
} BenmoDriveMotorPortOps;

typedef struct {
    const BenmoDriveMotorPortOps *ops;
    uint32_t feedback_timeout_ms;
} BenmoDriveMotorConfig;

typedef struct {
    /** 反馈转速，单位 RPM */
    int16_t speed_rpm;
    /** 协议保留的原始电流字段 */
    int16_t current_raw;
    /** 协议原始位置字段 */
    uint16_t position_raw;
    uint8_t error_code;
    uint8_t mode;
    uint32_t last_update_ms;
    uint32_t response_can_id;
    bool response_can_id_valid;
    bool valid;
} BenmoDriveMotorFeedback;

typedef struct {
    int16_t target_rpm;
    int16_t current_rpm;
    BenmoDriveMotorFeedback feedback;
} BenmoDriveMotorNode;

typedef struct {
    const BenmoDriveMotorPortOps *ops;
    BenmoDriveMotorNode nodes[BENMO_DRIVE_MOTOR_COUNT];
    uint8_t command_data[BENMO_DRIVE_MOTOR_CAN_DATA_LEN];
    uint8_t next_query_id;
    uint8_t last_query_id;
    uint32_t initialized_at_ms;
    uint32_t feedback_timeout_ms;
    uint32_t tx_error_count;
    bool awaiting_feedback;
    bool initialized;
} BenmoDriveMotor;

BenmoDriveMotorStatus benmo_drive_motor_init(BenmoDriveMotor *self,
                                             const BenmoDriveMotorConfig *config);
BenmoDriveMotorStatus benmo_drive_motor_set_target_rpm(BenmoDriveMotor *self,
                                                       uint8_t motor_id,
                                                       int16_t target_rpm);
BenmoDriveMotorStatus benmo_drive_motor_update(BenmoDriveMotor *self);
BenmoDriveMotorStatus benmo_drive_motor_stop_all(BenmoDriveMotor *self);
BenmoDriveMotorStatus benmo_drive_motor_request_next_feedback(BenmoDriveMotor *self);
BenmoDriveMotorStatus benmo_drive_motor_handle_feedback(BenmoDriveMotor *self,
                                                        uint32_t response_can_id,
                                                        const uint8_t *data,
                                                        uint8_t len);
BenmoDriveMotorStatus benmo_drive_motor_check_feedback(const BenmoDriveMotor *self);
/** 重新开始反馈超时观察窗口，不改变协议或电机使能状态 */
BenmoDriveMotorStatus benmo_drive_motor_reset_feedback_monitor(BenmoDriveMotor *self);
BenmoDriveMotorStatus benmo_drive_motor_get_feedback(const BenmoDriveMotor *self,
                                                     uint8_t motor_id,
                                                     BenmoDriveMotorFeedback *out);

#endif /* BENMO_DRIVE_MOTOR_H */
