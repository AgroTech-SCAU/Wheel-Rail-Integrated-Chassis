#include "rs06_steer_motor.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define RS06_TYPE_RUN 0x0100U
#define RS06_TYPE_ENABLE 0x0300U
#define RS06_TYPE_STOP 0x0400U
#define RS06_TYPE_SET_ZERO 0x0600U
#define RS06_TYPE_SET_ID 0x0701U
#define RS06_TYPE_WRITE_PARAM 0x1200U
#define RS06_TYPE_SAVE 0x1600U
#define RS06_PARAM_RUN_MODE 0x7005U
#define RS06_PARAM_POSITION_REFERENCE 0x7016U

static bool rs06_motor_id_valid(uint8_t motor_id)
{
    return motor_id > 0U;
}

static uint32_t rs06_extended_id(uint32_t type, uint8_t host_id, uint8_t motor_id)
{
    return (type << 16U) | ((uint32_t)host_id << 8U) | motor_id;
}

static uint16_t rs06_float_to_uint(float value, float minimum, float maximum, uint8_t bits)
{
    float span = maximum - minimum;
    float offset = value - minimum;
    if (offset < 0.0F) {
        offset = 0.0F;
    }
    if (offset > span) {
        offset = span;
    }
    return (uint16_t)((offset / span) * (float)((1UL << bits) - 1UL));
}

static Rs06SteerMotorStatus rs06_send(Rs06SteerMotor* self,
                                      uint32_t type,
                                      uint8_t host_id,
                                      uint8_t motor_id,
                                      const uint8_t data[8])
{
    if (self == NULL || !rs06_motor_id_valid(motor_id) || data == NULL) {
        return RS06_STEER_STATUS_INVALID_PARAM;
    }
    if (!self->initialized) {
        return RS06_STEER_STATUS_NOT_INITIALIZED;
    }
    return self->ops->send_extended(rs06_extended_id(type, host_id, motor_id), data, 8U)
               ? RS06_STEER_STATUS_OK
               : RS06_STEER_STATUS_PORT_ERROR;
}

Rs06SteerMotorStatus rs06_steer_motor_init(Rs06SteerMotor* self,
                                            const Rs06SteerMotorConfig* config)
{
    if (self == NULL || config == NULL || config->ops == NULL ||
        config->ops->send_extended == NULL || config->ops->delay_ms == NULL ||
        config->host_id == 0U) {
        return RS06_STEER_STATUS_INVALID_PARAM;
    }
    self->ops = config->ops;
    self->host_id = config->host_id;
    self->initialized = true;
    return RS06_STEER_STATUS_OK;
}

Rs06SteerMotorStatus rs06_steer_motor_enable(Rs06SteerMotor* self, uint8_t motor_id)
{
    const uint8_t data[8] = {0U};
    return rs06_send(self, RS06_TYPE_ENABLE, self != NULL ? self->host_id : 0U,
                     motor_id, data);
}

Rs06SteerMotorStatus rs06_steer_motor_stop(Rs06SteerMotor* self, uint8_t motor_id)
{
    const uint8_t data[8] = {0U};
    return rs06_send(self, RS06_TYPE_STOP, self != NULL ? self->host_id : 0U,
                     motor_id, data);
}

Rs06SteerMotorStatus rs06_steer_motor_change_id(Rs06SteerMotor* self,
                                                 uint8_t old_id,
                                                 uint8_t new_id)
{
    const uint8_t data[8] = {0U};
    if (!rs06_motor_id_valid(new_id)) {
        return RS06_STEER_STATUS_INVALID_PARAM;
    }
    return rs06_send(self, RS06_TYPE_SET_ID, new_id, old_id, data);
}

Rs06SteerMotorStatus rs06_steer_motor_set_mechanical_zero(Rs06SteerMotor* self,
                                                           uint8_t motor_id)
{
    const uint8_t data[8] = {1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    return rs06_send(self, RS06_TYPE_SET_ZERO, self != NULL ? self->host_id : 0U,
                     motor_id, data);
}

Rs06SteerMotorStatus rs06_steer_motor_save_config(Rs06SteerMotor* self,
                                                   uint8_t motor_id)
{
    const uint8_t data[8] = {0x01U, 0x02U, 0x03U, 0x04U,
                             0x05U, 0x06U, 0x07U, 0x08U};
    return rs06_send(self, RS06_TYPE_SAVE, self != NULL ? self->host_id : 0U,
                     motor_id, data);
}

Rs06SteerMotorStatus rs06_steer_motor_zero_and_save(Rs06SteerMotor* self,
                                                     uint8_t motor_id)
{
    Rs06SteerMotorStatus status = rs06_steer_motor_set_mechanical_zero(self, motor_id);
    if (status != RS06_STEER_STATUS_OK) {
        return status;
    }
    self->ops->delay_ms(100U);
    status = rs06_steer_motor_save_config(self, motor_id);
    self->ops->delay_ms(500U);
    return status;
}

Rs06SteerMotorStatus rs06_steer_motor_set_mode(Rs06SteerMotor* self,
                                                uint8_t motor_id,
                                                uint8_t mode)
{
    uint8_t data[8] = {0U};
    if (mode != RS06_STEER_MODE_MIT && mode != RS06_STEER_MODE_PP &&
        mode != RS06_STEER_MODE_CSP) {
        return RS06_STEER_STATUS_INVALID_PARAM;
    }
    data[0] = (uint8_t)RS06_PARAM_RUN_MODE;
    data[1] = (uint8_t)(RS06_PARAM_RUN_MODE >> 8U);
    data[4] = mode;
    return rs06_send(self, RS06_TYPE_WRITE_PARAM,
                     self != NULL ? self->host_id : 0U, motor_id, data);
}

Rs06SteerMotorStatus rs06_steer_motor_set_position_target(Rs06SteerMotor* self,
                                                           uint8_t motor_id,
                                                           float angle_rad)
{
    uint8_t data[8] = {0U};
    if (!isfinite(angle_rad) || angle_rad < RS06_STEER_POSITION_MIN_RAD ||
        angle_rad > RS06_STEER_POSITION_MAX_RAD) {
        return RS06_STEER_STATUS_INVALID_PARAM;
    }
    if (motor_id == 5U || motor_id == 7U) {
        angle_rad = -angle_rad;
    }
    data[0] = (uint8_t)RS06_PARAM_POSITION_REFERENCE;
    data[1] = (uint8_t)(RS06_PARAM_POSITION_REFERENCE >> 8U);
    memcpy(&data[4], &angle_rad, sizeof(angle_rad));
    return rs06_send(self, RS06_TYPE_WRITE_PARAM,
                     self != NULL ? self->host_id : 0U, motor_id, data);
}

Rs06SteerMotorStatus rs06_steer_motor_set_mit_position(Rs06SteerMotor* self,
                                                        uint8_t motor_id,
                                                        float angle_rad,
                                                        float speed_rad_s,
                                                        float kp,
                                                        float kd,
                                                        float torque_ff)
{
    uint16_t position;
    uint16_t velocity;
    uint16_t kp_encoded;
    uint16_t kd_encoded;
    uint16_t torque;
    uint8_t data[8];
    if (self == NULL || !rs06_motor_id_valid(motor_id) ||
        !isfinite(angle_rad) || !isfinite(speed_rad_s) || !isfinite(kp) ||
        !isfinite(kd) || !isfinite(torque_ff)) {
        return RS06_STEER_STATUS_INVALID_PARAM;
    }
    if (motor_id == 5U || motor_id == 7U) {
        angle_rad = -angle_rad;
        speed_rad_s = -speed_rad_s;
        torque_ff = -torque_ff;
    }
    position = rs06_float_to_uint(angle_rad, RS06_STEER_POSITION_MIN_RAD,
                                  RS06_STEER_POSITION_MAX_RAD, 16U);
    velocity = rs06_float_to_uint(speed_rad_s, RS06_STEER_VELOCITY_MIN_RAD_S,
                                  RS06_STEER_VELOCITY_MAX_RAD_S, 12U);
    kp_encoded = rs06_float_to_uint(kp, RS06_STEER_KP_MIN, RS06_STEER_KP_MAX, 12U);
    kd_encoded = rs06_float_to_uint(kd, RS06_STEER_KD_MIN, RS06_STEER_KD_MAX, 12U);
    torque = rs06_float_to_uint(torque_ff, -36.0F, 36.0F, 12U);
    data[0] = (uint8_t)(position >> 8U);
    data[1] = (uint8_t)position;
    data[2] = (uint8_t)(velocity >> 4U);
    data[3] = (uint8_t)(((velocity & 0x0FU) << 4U) | ((kp_encoded >> 8U) & 0x0FU));
    data[4] = (uint8_t)kp_encoded;
    data[5] = (uint8_t)(kd_encoded >> 4U);
    data[6] = (uint8_t)(((kd_encoded & 0x0FU) << 4U) | ((torque >> 8U) & 0x0FU));
    data[7] = (uint8_t)torque;
    return rs06_send(self, RS06_TYPE_RUN, self->host_id, motor_id, data);
}

bool rs06_steer_motor_is_initialized(const Rs06SteerMotor* self)
{
    return self != NULL && self->initialized;
}
