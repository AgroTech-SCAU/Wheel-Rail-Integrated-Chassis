#ifndef _rs06_steer_motor_h_
#define _rs06_steer_motor_h_

/**
 * @file rs06_steer_motor.h
 * @brief RS06 舵向电机协议驱动接口
 */

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

#define RS06_STEER_HOST_ID            0xFDu
#define RS06_STEER_MODE_MIT           0u
#define RS06_STEER_MODE_PP            1u
#define RS06_STEER_MODE_VELOCITY      2u
#define RS06_STEER_MODE_CSP           5u
#define RS06_STEER_POSITION_MIN_RAD   (-12.57f)
#define RS06_STEER_POSITION_MAX_RAD   12.57f
#define RS06_STEER_SAFE_POSITION_RAD  3.09159265f
#define RS06_STEER_VELOCITY_MIN_RAD_S (-50.0f)
#define RS06_STEER_VELOCITY_MAX_RAD_S 50.0f
#define RS06_STEER_KP_MIN             0.0f
#define RS06_STEER_KP_MAX             5000.0f
#define RS06_STEER_KD_MIN             0.0f
#define RS06_STEER_KD_MAX             100.0f

/**
 * @brief RS06 驱动状态码
 */
typedef enum {
    RS06_STEER_STATUS_OK = 0,
    RS06_STEER_STATUS_INVALID_PARAM,
    RS06_STEER_STATUS_NOT_INITIALIZED,
    RS06_STEER_STATUS_PORT_ERROR,
    RS06_STEER_STATUS_UNSUPPORTED_FRAME,
} Rs06SteerMotorStatus;

typedef struct {
    uint8_t motor_id;
    float angle_rad;
    float velocity_rad_s;
    float torque_nm;
    float temperature_c;
} Rs06SteerMotorFeedback;

/**
 * @brief RS06 平台能力接口
 */
typedef struct {
    bool (*send_extended)(uint32_t id, const uint8_t* data, uint8_t len);
    void (*delay_ms)(uint32_t delay_ms);
} Rs06SteerMotorPortOps;

/**
 * @brief RS06 初始化配置
 */
typedef struct {
    const Rs06SteerMotorPortOps* ops;
    uint8_t host_id;
} Rs06SteerMotorConfig;

/**
 * @brief RS06 驱动实例
 */
typedef struct {
    const Rs06SteerMotorPortOps* ops;
    uint8_t host_id;
    bool initialized;
} Rs06SteerMotor;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化 RS06 驱动实例
 * @param self 驱动实例
 * @param config 初始化配置
 * @return Rs06SteerMotorStatus 状态码
 */
Rs06SteerMotorStatus rs06_steer_motor_init(Rs06SteerMotor* self,
                                           const Rs06SteerMotorConfig* config);
/**
 * @brief 使能指定电机
 */
Rs06SteerMotorStatus rs06_steer_motor_enable(Rs06SteerMotor* self, uint8_t motor_id);
/**
 * @brief 停止指定电机
 */
Rs06SteerMotorStatus rs06_steer_motor_stop(Rs06SteerMotor* self, uint8_t motor_id);
/**
 * @brief 修改指定电机节点 ID
 */
Rs06SteerMotorStatus rs06_steer_motor_change_id(Rs06SteerMotor* self,
                                                uint8_t old_id,
                                                uint8_t new_id);
/**
 * @brief 设置指定电机运行模式
 */
Rs06SteerMotorStatus rs06_steer_motor_set_mode(Rs06SteerMotor* self,
                                               uint8_t motor_id,
                                               uint8_t mode);
/**
 * @brief 设置位置模式目标角
 */
Rs06SteerMotorStatus rs06_steer_motor_set_position_target(Rs06SteerMotor* self,
                                                          uint8_t motor_id,
                                                          float angle_rad);
Rs06SteerMotorStatus rs06_steer_motor_set_raw_position_target(
    Rs06SteerMotor* self, uint8_t motor_id, float angle_rad);
/**
 * @brief 设置速度模式目标速度
 */
Rs06SteerMotorStatus rs06_steer_motor_set_velocity_target(Rs06SteerMotor* self,
                                                          uint8_t motor_id,
                                                          float speed_rad_s);
/**
 * @brief 停止电机，写入无冲击 PP 目标后重新使能
 */
Rs06SteerMotorStatus rs06_steer_motor_prepare_pp(Rs06SteerMotor* self,
                                                 uint8_t motor_id,
                                                 float current_angle_rad);
Rs06SteerMotorStatus rs06_steer_motor_prepare_pp_raw(Rs06SteerMotor* self,
                                                     uint8_t motor_id,
                                                     float current_angle_rad);
/**
 * @brief 停止电机，清零速度目标后进入并使能速度模式
 */
Rs06SteerMotorStatus rs06_steer_motor_prepare_velocity(Rs06SteerMotor* self,
                                                       uint8_t motor_id);
/**
 * @brief 解析私有扩展帧协议的通信类型 2 电机反馈
 */
Rs06SteerMotorStatus rs06_steer_motor_parse_feedback(
    const Rs06SteerMotor* self, uint32_t extended_id, const uint8_t* data,
    uint8_t len, Rs06SteerMotorFeedback* out);
/**
 * @brief 设置 MIT 模式位置控制参数
 */
Rs06SteerMotorStatus rs06_steer_motor_set_mit_position(Rs06SteerMotor* self,
                                                       uint8_t motor_id,
                                                       float angle_rad,
                                                       float speed_rad_s,
                                                       float kp,
                                                       float kd,
                                                       float torque_ff);
/**
 * @brief 将当前位置设置为机械零位
 */
Rs06SteerMotorStatus rs06_steer_motor_set_mechanical_zero(Rs06SteerMotor* self,
                                                          uint8_t motor_id);
/**
 * @brief 保存电机配置
 */
Rs06SteerMotorStatus rs06_steer_motor_save_config(Rs06SteerMotor* self,
                                                  uint8_t motor_id);
/**
 * @brief 执行机械零位设置与配置保存流程
 */
Rs06SteerMotorStatus rs06_steer_motor_zero_and_save(Rs06SteerMotor* self,
                                                    uint8_t motor_id);
/**
 * @brief 判断驱动实例是否已初始化
 */
bool rs06_steer_motor_is_initialized(const Rs06SteerMotor* self);

#endif
