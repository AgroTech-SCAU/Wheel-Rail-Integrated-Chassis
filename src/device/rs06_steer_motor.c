/**
 * @file rs06_steer_motor.c
 * @brief RS06 舵向电机协议驱动实现
 */

#include "rs06_steer_motor.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

// ! ========================= 宏 定 义 声 明 ========================= ! //

#define RS06_TYPE_RUN 0x0100u
#define RS06_TYPE_ENABLE 0x0300u
#define RS06_TYPE_STOP 0x0400u
#define RS06_TYPE_SET_ZERO 0x0600u
#define RS06_TYPE_SET_ID 0x0701u
#define RS06_TYPE_WRITE_PARAM 0x1200u
#define RS06_TYPE_SAVE 0x1600u
#define RS06_PARAM_RUN_MODE 0x7005u
#define RS06_PARAM_VELOCITY_REFERENCE 0x700Au
#define RS06_PARAM_POSITION_REFERENCE 0x7016u
#define RS06_TYPE_FEEDBACK 0x02u

static Rs06SteerMotorStatus rs06_send(Rs06SteerMotor* self, uint32_t type,
                                      uint8_t host_id, uint8_t motor_id,
                                      const uint8_t data[8]);

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 判断 RS06 节点 ID 是否有效
 */
static bool rs06_motor_id_valid(uint8_t motor_id) {
    return motor_id > 0u;
}

/**
 * @brief 按旧协议组合 RS06 扩展帧 ID
 */
static uint32_t rs06_extended_id(uint32_t type, uint8_t host_id,
                                 uint8_t motor_id) {
    return (type << 16u) | ((uint32_t)host_id << 8u) | motor_id;
}

/**
 * @brief 将浮点控制量映射为指定位宽无符号整数
 */
static uint16_t rs06_float_to_uint(float value, float minimum, float maximum,
                                   uint8_t bits) {
    float span = maximum - minimum;
    float offset = value - minimum;
    if(offset < 0.0f) {
        offset = 0.0f;
    }
    if(offset > span) {
        offset = span;
    }
    return (uint16_t)((offset / span) * (float)((1ul << bits) - 1ul));
}

static float rs06_uint_to_float(uint16_t value, float minimum, float maximum) {
    return ((float)value * (maximum - minimum) / 65535.0f) + minimum;
}

static Rs06SteerMotorStatus rs06_write_float_param(Rs06SteerMotor* self,
                                                   uint8_t motor_id,
                                                   uint16_t index,
                                                   float value) {
    uint8_t data[8] = { 0u };
    if(!isfinite(value)) {
        return RS06_STEER_STATUS_INVALID_PARAM;
    }
    data[0] = (uint8_t)index;
    data[1] = (uint8_t)(index >> 8u);
    memcpy(&data[4], &value, sizeof(value));
    return rs06_send(self, RS06_TYPE_WRITE_PARAM,
                     self != NULL ? self->host_id : 0u, motor_id, data);
}

/**
 * @brief 发送一帧 RS06 协议数据
 */
static Rs06SteerMotorStatus rs06_send(Rs06SteerMotor* self, uint32_t type,
                                      uint8_t host_id, uint8_t motor_id,
                                      const uint8_t data[8]) {
    if(self == NULL || !rs06_motor_id_valid(motor_id) || data == NULL) {
        return RS06_STEER_STATUS_INVALID_PARAM;
    }
    if(!self->initialized) {
        return RS06_STEER_STATUS_NOT_INITIALIZED;
    }
    return self->ops->send_extended(rs06_extended_id(type, host_id, motor_id),
                                    data, 8u)
               ? RS06_STEER_STATUS_OK
               : RS06_STEER_STATUS_PORT_ERROR;
}

// ! ========================= 接 口 函 数 实 现 ========================= ! //

Rs06SteerMotorStatus rs06_steer_motor_init(Rs06SteerMotor* self,
                                           const Rs06SteerMotorConfig* config) {
    if(self == NULL || config == NULL || config->ops == NULL ||
       config->ops->send_extended == NULL || config->ops->delay_ms == NULL ||
       config->host_id == 0u) {
        return RS06_STEER_STATUS_INVALID_PARAM;
    }
    self->ops = config->ops;
    self->host_id = config->host_id;
    self->initialized = true;
    return RS06_STEER_STATUS_OK;
}

Rs06SteerMotorStatus rs06_steer_motor_enable(Rs06SteerMotor* self,
                                             uint8_t motor_id) {
    const uint8_t data[8] = { 0u };
    return rs06_send(self, RS06_TYPE_ENABLE, self != NULL ? self->host_id : 0u,
                     motor_id, data);
}

Rs06SteerMotorStatus rs06_steer_motor_stop(Rs06SteerMotor* self,
                                           uint8_t motor_id) {
    const uint8_t data[8] = { 0u };
    return rs06_send(self, RS06_TYPE_STOP, self != NULL ? self->host_id : 0u,
                     motor_id, data);
}

Rs06SteerMotorStatus rs06_steer_motor_change_id(Rs06SteerMotor* self,
                                                uint8_t old_id,
                                                uint8_t new_id) {
    const uint8_t data[8] = { 0u };
    if(!rs06_motor_id_valid(new_id)) {
        return RS06_STEER_STATUS_INVALID_PARAM;
    }
    return rs06_send(self, RS06_TYPE_SET_ID, new_id, old_id, data);
}

Rs06SteerMotorStatus rs06_steer_motor_set_mechanical_zero(Rs06SteerMotor* self,
                                                          uint8_t motor_id) {
    const uint8_t data[8] = { 1u, 0u, 0u, 0u, 0u, 0u, 0u, 0u };
    return rs06_send(self, RS06_TYPE_SET_ZERO, self != NULL ? self->host_id : 0u,
                     motor_id, data);
}

Rs06SteerMotorStatus rs06_steer_motor_save_config(Rs06SteerMotor* self,
                                                  uint8_t motor_id) {
    const uint8_t data[8] = { 0x01u, 0x02u, 0x03u, 0x04u,
                              0x05u, 0x06u, 0x07u, 0x08u };
    return rs06_send(self, RS06_TYPE_SAVE, self != NULL ? self->host_id : 0u,
                     motor_id, data);
}

Rs06SteerMotorStatus rs06_steer_motor_zero_and_save(Rs06SteerMotor* self,
                                                    uint8_t motor_id) {
    Rs06SteerMotorStatus status =
        rs06_steer_motor_set_mechanical_zero(self, motor_id);
    if(status != RS06_STEER_STATUS_OK) {
        return status;
    }
    self->ops->delay_ms(100u);
    status = rs06_steer_motor_save_config(self, motor_id);
    self->ops->delay_ms(500u);
    return status;
}

Rs06SteerMotorStatus rs06_steer_motor_set_mode(Rs06SteerMotor* self,
                                               uint8_t motor_id, uint8_t mode) {
    uint8_t data[8] = { 0u };
    if(mode != RS06_STEER_MODE_MIT && mode != RS06_STEER_MODE_PP &&
       mode != RS06_STEER_MODE_VELOCITY &&
       mode != RS06_STEER_MODE_CSP) {
        return RS06_STEER_STATUS_INVALID_PARAM;
    }
    data[0] = (uint8_t)RS06_PARAM_RUN_MODE;
    data[1] = (uint8_t)(RS06_PARAM_RUN_MODE >> 8u);
    data[4] = mode;
    return rs06_send(self, RS06_TYPE_WRITE_PARAM,
                     self != NULL ? self->host_id : 0u, motor_id, data);
}

Rs06SteerMotorStatus rs06_steer_motor_set_position_target(Rs06SteerMotor* self,
                                                          uint8_t motor_id,
                                                          float angle_rad) {
    if(!isfinite(angle_rad) || angle_rad < -RS06_STEER_SAFE_POSITION_RAD ||
       angle_rad > RS06_STEER_SAFE_POSITION_RAD) {
        return RS06_STEER_STATUS_INVALID_PARAM;
    }
    return rs06_steer_motor_set_raw_position_target(self, motor_id, angle_rad);
}

Rs06SteerMotorStatus rs06_steer_motor_set_raw_position_target(
    Rs06SteerMotor* self, uint8_t motor_id, float angle_rad) {
    if(!isfinite(angle_rad) || angle_rad < RS06_STEER_POSITION_MIN_RAD ||
       angle_rad > RS06_STEER_POSITION_MAX_RAD) {
        return RS06_STEER_STATUS_INVALID_PARAM;
    }
    if(motor_id == 5u || motor_id == 7u) {
        angle_rad = -angle_rad;
    }
    return rs06_write_float_param(self, motor_id, RS06_PARAM_POSITION_REFERENCE,
                                  angle_rad);
}

Rs06SteerMotorStatus rs06_steer_motor_set_velocity_target(Rs06SteerMotor* self,
                                                          uint8_t motor_id,
                                                          float speed_rad_s) {
    if(!isfinite(speed_rad_s) || speed_rad_s < RS06_STEER_VELOCITY_MIN_RAD_S ||
       speed_rad_s > RS06_STEER_VELOCITY_MAX_RAD_S) {
        return RS06_STEER_STATUS_INVALID_PARAM;
    }
    if(motor_id == 5u || motor_id == 7u) {
        speed_rad_s = -speed_rad_s;
    }
    return rs06_write_float_param(self, motor_id, RS06_PARAM_VELOCITY_REFERENCE,
                                  speed_rad_s);
}

Rs06SteerMotorStatus rs06_steer_motor_prepare_pp(Rs06SteerMotor* self,
                                                 uint8_t motor_id,
                                                 float current_angle_rad) {
    Rs06SteerMotorStatus status;
    status = rs06_steer_motor_set_velocity_target(self, motor_id, 0.0f);
    if(status != RS06_STEER_STATUS_OK) {
        return status;
    }
    self->ops->delay_ms(5u);
    status = rs06_steer_motor_stop(self, motor_id);
    if(status != RS06_STEER_STATUS_OK) {
        return status;
    }
    self->ops->delay_ms(5u);
    status = rs06_steer_motor_set_mode(self, motor_id, RS06_STEER_MODE_PP);
    if(status != RS06_STEER_STATUS_OK) {
        return status;
    }
    self->ops->delay_ms(5u);
    status =
        rs06_steer_motor_set_position_target(self, motor_id, current_angle_rad);
    if(status != RS06_STEER_STATUS_OK) {
        return status;
    }
    self->ops->delay_ms(5u);
    return rs06_steer_motor_enable(self, motor_id);
}

Rs06SteerMotorStatus rs06_steer_motor_prepare_pp_raw(Rs06SteerMotor* self,
                                                     uint8_t motor_id,
                                                     float current_angle_rad) {
    Rs06SteerMotorStatus status = rs06_steer_motor_stop(self, motor_id);
    if(status != RS06_STEER_STATUS_OK) return status;
    self->ops->delay_ms(5u);
    status = rs06_steer_motor_set_mode(self, motor_id, RS06_STEER_MODE_PP);
    if(status != RS06_STEER_STATUS_OK) return status;
    self->ops->delay_ms(5u);
    status = rs06_steer_motor_set_raw_position_target(self, motor_id,
                                                      current_angle_rad);
    if(status != RS06_STEER_STATUS_OK) return status;
    self->ops->delay_ms(5u);
    return rs06_steer_motor_enable(self, motor_id);
}

Rs06SteerMotorStatus rs06_steer_motor_prepare_velocity(Rs06SteerMotor* self,
                                                       uint8_t motor_id) {
    Rs06SteerMotorStatus status = rs06_steer_motor_stop(self, motor_id);
    if(status != RS06_STEER_STATUS_OK) {
        return status;
    }
    self->ops->delay_ms(5u);
    status =
        rs06_steer_motor_set_mode(self, motor_id, RS06_STEER_MODE_VELOCITY);
    if(status != RS06_STEER_STATUS_OK) {
        return status;
    }
    self->ops->delay_ms(5u);
    status = rs06_steer_motor_set_velocity_target(self, motor_id, 0.0f);
    if(status != RS06_STEER_STATUS_OK) {
        return status;
    }
    self->ops->delay_ms(5u);
    return rs06_steer_motor_enable(self, motor_id);
}

Rs06SteerMotorStatus rs06_steer_motor_parse_feedback(
    const Rs06SteerMotor* self, uint32_t extended_id, const uint8_t* data,
    uint8_t len, Rs06SteerMotorFeedback* out) {
    uint8_t motor_id;
    uint16_t angle_raw;
    uint16_t velocity_raw;
    uint16_t torque_raw;
    uint16_t temperature_raw;
    if(self == NULL || !self->initialized || data == NULL || out == NULL ||
       len != 8u) {
        return RS06_STEER_STATUS_INVALID_PARAM;
    }
    if(((extended_id >> 24u) & 0x1Fu) != RS06_TYPE_FEEDBACK ||
       (uint8_t)extended_id != self->host_id) {
        return RS06_STEER_STATUS_UNSUPPORTED_FRAME;
    }
    motor_id = (uint8_t)(extended_id >> 8u);
    if(!rs06_motor_id_valid(motor_id)) {
        return RS06_STEER_STATUS_INVALID_PARAM;
    }
    angle_raw = (uint16_t)(((uint16_t)data[0] << 8u) | data[1]);
    velocity_raw = (uint16_t)(((uint16_t)data[2] << 8u) | data[3]);
    torque_raw = (uint16_t)(((uint16_t)data[4] << 8u) | data[5]);
    temperature_raw = (uint16_t)(((uint16_t)data[6] << 8u) | data[7]);
    out->motor_id = motor_id;
    out->angle_rad = rs06_uint_to_float(angle_raw, RS06_STEER_POSITION_MIN_RAD,
                                        RS06_STEER_POSITION_MAX_RAD);
    out->velocity_rad_s =
        rs06_uint_to_float(velocity_raw, RS06_STEER_VELOCITY_MIN_RAD_S,
                           RS06_STEER_VELOCITY_MAX_RAD_S);
    out->torque_nm = rs06_uint_to_float(torque_raw, -36.0f, 36.0f);
    out->temperature_c = (float)temperature_raw * 0.1f;
    if(motor_id == 5u || motor_id == 7u) {
        out->angle_rad = -out->angle_rad;
        out->velocity_rad_s = -out->velocity_rad_s;
        out->torque_nm = -out->torque_nm;
    }
    return RS06_STEER_STATUS_OK;
}

Rs06SteerMotorStatus
rs06_steer_motor_set_mit_position(Rs06SteerMotor* self, uint8_t motor_id,
                                  float angle_rad, float speed_rad_s, float kp,
                                  float kd, float torque_ff) {
    uint16_t position;
    uint16_t velocity;
    uint16_t kp_encoded;
    uint16_t kd_encoded;
    uint16_t torque;
    uint8_t data[8];
    if(self == NULL || !rs06_motor_id_valid(motor_id) || !isfinite(angle_rad) ||
       !isfinite(speed_rad_s) || !isfinite(kp) || !isfinite(kd) ||
       !isfinite(torque_ff)) {
        return RS06_STEER_STATUS_INVALID_PARAM;
    }
    if(motor_id == 5u || motor_id == 7u) {
        angle_rad = -angle_rad;
        speed_rad_s = -speed_rad_s;
        torque_ff = -torque_ff;
    }
    position = rs06_float_to_uint(angle_rad, RS06_STEER_POSITION_MIN_RAD,
                                  RS06_STEER_POSITION_MAX_RAD, 16u);
    velocity = rs06_float_to_uint(speed_rad_s, RS06_STEER_VELOCITY_MIN_RAD_S,
                                  RS06_STEER_VELOCITY_MAX_RAD_S, 16u);
    kp_encoded =
        rs06_float_to_uint(kp, RS06_STEER_KP_MIN, RS06_STEER_KP_MAX, 16u);
    kd_encoded =
        rs06_float_to_uint(kd, RS06_STEER_KD_MIN, RS06_STEER_KD_MAX, 16u);
    torque = rs06_float_to_uint(torque_ff, -36.0f, 36.0f, 16u);
    data[0] = (uint8_t)(position >> 8u);
    data[1] = (uint8_t)position;
    data[2] = (uint8_t)(velocity >> 8u);
    data[3] = (uint8_t)velocity;
    data[4] = (uint8_t)(kp_encoded >> 8u);
    data[5] = (uint8_t)kp_encoded;
    data[6] = (uint8_t)(kd_encoded >> 8u);
    data[7] = (uint8_t)kd_encoded;
    return rs06_send(self, RS06_TYPE_RUN | (uint32_t)(torque >> 8u),
                     (uint8_t)torque, motor_id, data);
}

bool rs06_steer_motor_is_initialized(const Rs06SteerMotor* self) {
    return self != NULL && self->initialized;
}
