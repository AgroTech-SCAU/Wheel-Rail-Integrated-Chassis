#ifndef __SWERVE_CHASSIS_H__
#define __SWERVE_CHASSIS_H__

#include "steer_wheel_kine.h"
#include "motor.h"
#include "servo.h"
#include "fdcan.h"

/* ---------------- 舵轮底盘结构体 ---------------- */

typedef struct
{
    /* 运动学实例 */
    SteerWheel kine;

    /* 底盘物理模型参数 */
    SteerWheelModel model;

    /* 驱动轮 ID */
    uint8_t drive_ids[4];

    /* 舵向轮 ID */
    uint8_t steer_ids[4];

} SwerveChassis;

/* ---------------- 接口函数 ---------------- */

/**
 * @brief 初始化底盘物理参数
 * @param length 底盘前后轴距(m)
 * @param width 底盘左右轮距(m)
 * @param wheel_radius 轮子半径(m)
 * @param max_speed 单轮最大线速度(m/s)
 */
void Swerve_Chassis_Model_Init(
    SwerveChassis* chassis,
    float length,
    float width,
    float wheel_radius,
    float max_speed
);

/**
 * @brief 初始化舵轮底盘
 */
void Swerve_Chassis_Init(SwerveChassis* chassis);

/**
 * @brief 设置底盘速度
 * @param vx 前进速度 m/s
 * @param vy 横移速度 m/s
 * @param wz 自旋角速度 rad/s
 */
void Swerve_Chassis_Set_Velocity(
    SwerveChassis* chassis,
    float vx,
    float vy,
    float wz
);

/**
 * @brief 更新底盘执行
 * @note 内部自动调用 IK + 电机输出
 */
void Swerve_Chassis_Update(SwerveChassis* chassis);

#endif /* __SWERVE_CHASSIS_H__ */