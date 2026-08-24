/**
 * @file benmo_drive_motor.c
 * @brief 本末驱动电机协议驱动实现
 */

#include "benmo_drive_motor.h"

#include <stddef.h>
#include <string.h>

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 判断节点 ID 是否处于驱动电机有效范围
 */
static bool benmo_drive_id_valid(uint8_t motor_id) {
    return motor_id >= BENMO_DRIVE_MOTOR_ID_MIN &&
           motor_id <= BENMO_DRIVE_MOTOR_ID_MAX;
}

/**
 * @brief 将目标转速限制在协议允许范围内
 */
static int16_t benmo_drive_limit_rpm(int16_t rpm) {
    if(rpm > BENMO_DRIVE_MOTOR_RPM_MAX) {
        return BENMO_DRIVE_MOTOR_RPM_MAX;
    }
    if(rpm < BENMO_DRIVE_MOTOR_RPM_MIN) {
        return BENMO_DRIVE_MOTOR_RPM_MIN;
    }
    return rpm;
}

/**
 * @brief 将单节点转速写入四节点组合命令缓存
 */
static void benmo_drive_encode(BenmoDriveMotor* self, uint8_t motor_id,
                               int16_t rpm) {
    int32_t scaled;
    uint8_t index;
    if(motor_id == 3u || motor_id == 4u) {
        rpm = (int16_t)-rpm;
    }
    scaled = (int32_t)rpm * 100;
    if(scaled > 32767) {
        scaled = 32767;
    }
    else if(scaled < -32767) {
        scaled = -32767;
    }
    index = (uint8_t)((motor_id - BENMO_DRIVE_MOTOR_ID_MIN) * 2u);
    self->command_data[index] = (uint8_t)(((uint16_t)(int16_t)scaled) >> 8u);
    self->command_data[index + 1u] = (uint8_t)scaled;
}

// ! ========================= 接 口 函 数 实 现 ========================= ! //

BenmoDriveMotorStatus
benmo_drive_motor_init(BenmoDriveMotor* self,
                       const BenmoDriveMotorConfig* config) {
    if(self == NULL || config == NULL || config->ops == NULL ||
       config->ops->send_standard == NULL || config->ops->now_ms == NULL ||
       config->ops->critical_enter == NULL ||
       config->ops->critical_exit == NULL || config->feedback_timeout_ms == 0u) {
        return BENMO_DRIVE_STATUS_INVALID_PARAM;
    }
    memset(self, 0, sizeof(*self));
    self->ops = config->ops;
    self->feedback_timeout_ms = config->feedback_timeout_ms;
    self->next_query_id = BENMO_DRIVE_MOTOR_ID_MIN;
    self->last_query_id = BENMO_DRIVE_MOTOR_ID_MIN;
    self->initialized_at_ms = self->ops->now_ms();
    self->initialized = true;
    return BENMO_DRIVE_STATUS_OK;
}

BenmoDriveMotorStatus benmo_drive_motor_set_target_rpm(BenmoDriveMotor* self,
                                                       uint8_t motor_id,
                                                       int16_t target_rpm) {
    if(self == NULL || !benmo_drive_id_valid(motor_id)) {
        return BENMO_DRIVE_STATUS_INVALID_PARAM;
    }
    if(!self->initialized) {
        return BENMO_DRIVE_STATUS_NOT_INITIALIZED;
    }
    self->nodes[motor_id - BENMO_DRIVE_MOTOR_ID_MIN].target_rpm =
        benmo_drive_limit_rpm(target_rpm);
    return BENMO_DRIVE_STATUS_OK;
}

BenmoDriveMotorStatus benmo_drive_motor_update(BenmoDriveMotor* self) {
    uint8_t i;
    uint32_t critical_state;
    if(self == NULL) {
        return BENMO_DRIVE_STATUS_INVALID_PARAM;
    }
    if(!self->initialized) {
        return BENMO_DRIVE_STATUS_NOT_INITIALIZED;
    }

    critical_state = self->ops->critical_enter();
    for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        int16_t target = self->nodes[i].target_rpm;
        int16_t current = self->nodes[i].current_rpm;
        int32_t difference = (int32_t)target - (int32_t)current;
        if(difference > BENMO_DRIVE_MOTOR_RAMP_STEP_RPM) {
            current = (int16_t)(current + BENMO_DRIVE_MOTOR_RAMP_STEP_RPM);
        }
        else if(difference < -BENMO_DRIVE_MOTOR_RAMP_STEP_RPM) {
            current = (int16_t)(current - BENMO_DRIVE_MOTOR_RAMP_STEP_RPM);
        }
        else {
            current = target;
        }
        self->nodes[i].current_rpm = current;
        benmo_drive_encode(self, (uint8_t)(i + BENMO_DRIVE_MOTOR_ID_MIN), current);
    }
    self->ops->critical_exit(critical_state);
    if(!self->ops->send_standard(BENMO_DRIVE_MOTOR_SPEED_COMMAND_CAN_ID,
                                 self->command_data,
                                 BENMO_DRIVE_MOTOR_CAN_DATA_LEN)) {
        self->tx_error_count++;
        return BENMO_DRIVE_STATUS_PORT_ERROR;
    }
    return BENMO_DRIVE_STATUS_OK;
}

BenmoDriveMotorStatus benmo_drive_motor_stop_all(BenmoDriveMotor* self) {
    uint8_t i;
    uint32_t critical_state;
    if(self == NULL) {
        return BENMO_DRIVE_STATUS_INVALID_PARAM;
    }
    if(!self->initialized) {
        return BENMO_DRIVE_STATUS_NOT_INITIALIZED;
    }
    critical_state = self->ops->critical_enter();
    for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        self->nodes[i].target_rpm = 0;
        self->nodes[i].current_rpm = 0;
        benmo_drive_encode(self, (uint8_t)(i + BENMO_DRIVE_MOTOR_ID_MIN), 0);
    }
    self->ops->critical_exit(critical_state);
    if(!self->ops->send_standard(BENMO_DRIVE_MOTOR_SPEED_COMMAND_CAN_ID,
                                 self->command_data,
                                 BENMO_DRIVE_MOTOR_CAN_DATA_LEN)) {
        self->tx_error_count++;
        return BENMO_DRIVE_STATUS_PORT_ERROR;
    }
    return BENMO_DRIVE_STATUS_OK;
}

BenmoDriveMotorStatus
benmo_drive_motor_request_next_feedback(BenmoDriveMotor* self) {
    uint8_t data[BENMO_DRIVE_MOTOR_CAN_DATA_LEN] = { 0u };
    if(self == NULL) {
        return BENMO_DRIVE_STATUS_INVALID_PARAM;
    }
    if(!self->initialized) {
        return BENMO_DRIVE_STATUS_NOT_INITIALIZED;
    }
    if(self->awaiting_feedback) {
        return BENMO_DRIVE_STATUS_OK;
    }
    self->last_query_id = self->next_query_id;
    data[0] = self->last_query_id;
    data[1] = 1u;
    data[2] = 4u;
    data[3] = 5u;
    data[4] = 0xAAu;
    if(!self->ops->send_standard(BENMO_DRIVE_MOTOR_QUERY_CAN_ID, data,
                                 BENMO_DRIVE_MOTOR_CAN_DATA_LEN)) {
        self->tx_error_count++;
        return BENMO_DRIVE_STATUS_PORT_ERROR;
    }
    self->awaiting_feedback = true;
    return BENMO_DRIVE_STATUS_OK;
}

BenmoDriveMotorStatus
benmo_drive_motor_handle_feedback(BenmoDriveMotor* self,
                                  uint32_t response_can_id, const uint8_t* data,
                                  uint8_t len) {
    BenmoDriveMotorFeedback* feedback;
    if(self == NULL || data == NULL || len != BENMO_DRIVE_MOTOR_CAN_DATA_LEN) {
        return BENMO_DRIVE_STATUS_INVALID_PARAM;
    }
    if(!self->initialized) {
        return BENMO_DRIVE_STATUS_NOT_INITIALIZED;
    }
    if(!self->awaiting_feedback || !benmo_drive_id_valid(self->last_query_id)) {
        return BENMO_DRIVE_STATUS_INVALID_PARAM;
    }
    feedback =
        &self->nodes[self->last_query_id - BENMO_DRIVE_MOTOR_ID_MIN].feedback;
    /* 旧协议未提供固定响应 ID 表；FDCAN1 为专用驱动总线，首次响应建立节点 ID 绑定
   */
    if(feedback->response_can_id_valid &&
       feedback->response_can_id != response_can_id) {
        return BENMO_DRIVE_STATUS_INVALID_PARAM;
    }
    feedback->speed_rpm = (int16_t)(((uint16_t)data[0] << 8u) | data[1]);
    feedback->position_raw = (uint16_t)(((uint16_t)data[2] << 8u) | data[3]);
    feedback->error_code = data[4];
    feedback->last_update_ms = self->ops->now_ms();
    feedback->response_can_id = response_can_id;
    feedback->response_can_id_valid = true;
    feedback->valid = true;
    self->awaiting_feedback = false;
    self->next_query_id = (uint8_t)(self->last_query_id + 1u);
    if(self->next_query_id > BENMO_DRIVE_MOTOR_ID_MAX) {
        self->next_query_id = BENMO_DRIVE_MOTOR_ID_MIN;
    }
    return BENMO_DRIVE_STATUS_OK;
}

BenmoDriveMotorStatus
benmo_drive_motor_check_feedback(const BenmoDriveMotor* self) {
    uint8_t i;
    uint32_t now;
    if(self == NULL) {
        return BENMO_DRIVE_STATUS_INVALID_PARAM;
    }
    if(!self->initialized) {
        return BENMO_DRIVE_STATUS_NOT_INITIALIZED;
    }
    now = self->ops->now_ms();
    for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        const BenmoDriveMotorFeedback* feedback = &self->nodes[i].feedback;
        uint32_t reference =
            feedback->valid ? feedback->last_update_ms : self->initialized_at_ms;
        if((uint32_t)(now - reference) >= self->feedback_timeout_ms) {
            return BENMO_DRIVE_STATUS_FEEDBACK_TIMEOUT;
        }
    }
    return BENMO_DRIVE_STATUS_OK;
}

BenmoDriveMotorStatus
benmo_drive_motor_reset_feedback_monitor(BenmoDriveMotor* self) {
    uint8_t i;
    uint32_t critical_state;
    if(self == NULL) {
        return BENMO_DRIVE_STATUS_INVALID_PARAM;
    }
    if(!self->initialized) {
        return BENMO_DRIVE_STATUS_NOT_INITIALIZED;
    }
    critical_state = self->ops->critical_enter();
    for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        self->nodes[i].feedback.valid = false;
        self->nodes[i].feedback.response_can_id_valid = false;
    }
    self->next_query_id = BENMO_DRIVE_MOTOR_ID_MIN;
    self->last_query_id = BENMO_DRIVE_MOTOR_ID_MIN;
    self->awaiting_feedback = false;
    self->initialized_at_ms = self->ops->now_ms();
    self->ops->critical_exit(critical_state);
    return BENMO_DRIVE_STATUS_OK;
}

BenmoDriveMotorStatus
benmo_drive_motor_get_feedback(const BenmoDriveMotor* self, uint8_t motor_id,
                               BenmoDriveMotorFeedback* out) {
    uint32_t critical_state;
    if(self == NULL || out == NULL || !benmo_drive_id_valid(motor_id)) {
        return BENMO_DRIVE_STATUS_INVALID_PARAM;
    }
    if(!self->initialized) {
        return BENMO_DRIVE_STATUS_NOT_INITIALIZED;
    }
    critical_state = self->ops->critical_enter();
    *out = self->nodes[motor_id - BENMO_DRIVE_MOTOR_ID_MIN].feedback;
    self->ops->critical_exit(critical_state);
    return BENMO_DRIVE_STATUS_OK;
}
