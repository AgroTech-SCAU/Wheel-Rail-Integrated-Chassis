#ifndef RS06_STEER_MOTOR_H
#define RS06_STEER_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#define RS06_STEER_HOST_ID            0xFDU
#define RS06_STEER_MODE_MIT           0U
#define RS06_STEER_MODE_PP            1U
#define RS06_STEER_MODE_CSP           5U
#define RS06_STEER_POSITION_MIN_RAD   (-12.57F)
#define RS06_STEER_POSITION_MAX_RAD   12.57F
#define RS06_STEER_VELOCITY_MIN_RAD_S (-50.0F)
#define RS06_STEER_VELOCITY_MAX_RAD_S 50.0F
#define RS06_STEER_KP_MIN             0.0F
#define RS06_STEER_KP_MAX             5000.0F
#define RS06_STEER_KD_MIN             0.0F
#define RS06_STEER_KD_MAX             100.0F

/** RS06 舵向电机协议驱动；角度使用 rad，角速度使用 rad/s */

typedef enum {
    RS06_STEER_STATUS_OK = 0,
    RS06_STEER_STATUS_INVALID_PARAM,
    RS06_STEER_STATUS_NOT_INITIALIZED,
    RS06_STEER_STATUS_PORT_ERROR,
} Rs06SteerMotorStatus;

typedef struct {
    bool (*send_extended)(uint32_t id, const uint8_t *data, uint8_t len);
    void (*delay_ms)(uint32_t delay_ms);
} Rs06SteerMotorPortOps;

typedef struct {
    const Rs06SteerMotorPortOps *ops;
    uint8_t host_id;
} Rs06SteerMotorConfig;

typedef struct {
    const Rs06SteerMotorPortOps *ops;
    uint8_t host_id;
    bool initialized;
} Rs06SteerMotor;

Rs06SteerMotorStatus rs06_steer_motor_init(Rs06SteerMotor *self,
                                           const Rs06SteerMotorConfig *config);
Rs06SteerMotorStatus rs06_steer_motor_enable(Rs06SteerMotor *self, uint8_t motor_id);
Rs06SteerMotorStatus rs06_steer_motor_stop(Rs06SteerMotor *self, uint8_t motor_id);
Rs06SteerMotorStatus rs06_steer_motor_change_id(Rs06SteerMotor *self,
                                                uint8_t old_id,
                                                uint8_t new_id);
Rs06SteerMotorStatus rs06_steer_motor_set_mode(Rs06SteerMotor *self,
                                               uint8_t motor_id,
                                               uint8_t mode);
Rs06SteerMotorStatus rs06_steer_motor_set_position_target(Rs06SteerMotor *self,
                                                          uint8_t motor_id,
                                                          float angle_rad);
Rs06SteerMotorStatus rs06_steer_motor_set_mit_position(Rs06SteerMotor *self,
                                                       uint8_t motor_id,
                                                       float angle_rad,
                                                       float speed_rad_s,
                                                       float kp,
                                                       float kd,
                                                       float torque_ff);
Rs06SteerMotorStatus rs06_steer_motor_set_mechanical_zero(Rs06SteerMotor *self,
                                                          uint8_t motor_id);
Rs06SteerMotorStatus rs06_steer_motor_save_config(Rs06SteerMotor *self,
                                                  uint8_t motor_id);
Rs06SteerMotorStatus rs06_steer_motor_zero_and_save(Rs06SteerMotor *self,
                                                    uint8_t motor_id);
bool rs06_steer_motor_is_initialized(const Rs06SteerMotor *self);

#endif /* RS06_STEER_MOTOR_H */
