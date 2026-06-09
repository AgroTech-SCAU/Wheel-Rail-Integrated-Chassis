#ifndef __SWERVE_CHASSIS_H__
#define __SWERVE_CHASSIS_H__

#include "steer_wheel_kine.h"
#include "motor.h"
#include "servo.h"
#include "fdcan.h"
#include <stdint.h>

/* ---------------- 舵轮底盘结构体 ---------------- */

typedef struct
{
    /* 运动学实例 */
    SteerWheel kine;

    /* 底盘物理模型参数 */
    SteerWheelModel model;

    /* 驱动轮 ID
     *
     * 驱动轮 ID：1~4
     */
    uint8_t drive_ids[4];

    /* 舵向轮 ID
     *
     * 舵向电机 ID：5~8
     */
    uint8_t steer_ids[4];

} SwerveChassis;


/* ---------------- 接口函数 ---------------- */

/**
 * @brief 初始化底盘物理参数
 *
 * @param chassis      舵轮底盘对象指针
 * @param length       底盘前后轴距，单位 m
 * @param width        底盘左右轮距，单位 m
 * @param wheel_radius 轮子半径，单位 m
 * @param max_speed    单轮最大线速度，单位 m/s
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
 *
 * @note 内部会完成：
 *       1. 初始化运动学模型
 *       2. 设置驱动轮 ID：1~4
 *       3. 设置舵向轮 ID：5~8
 *       4. 调用 Motor_Driver_Init()
 *       5. 设置舵向电机 PP 模式
 *       6. 使能舵向电机
 *       7. 给舵向电机发送 0 位置目标
 */
void Swerve_Chassis_Init(SwerveChassis* chassis);


/**
 * @brief 设置底盘目标速度
 *
 * @param chassis 舵轮底盘对象指针
 * @param vx      前进速度
 * @param vy      横移速度
 * @param wz      自旋角速度
 *
 * @note 当前你的 remote_chassis.c 中，vx / vy / wz 最大可为 ±20。
 */
void Swerve_Chassis_Set_Velocity(
    SwerveChassis* chassis,
    float vx,
    float vy,
    float wz
);


/**
 * @brief 更新舵轮底盘执行
 *
 * @note 内部会完成：
 *       1. IK 逆运动学解算
 *       2. 驱动轮速度输出
 *       3. 舵向轮设置 PP 模式
 *       4. 舵向轮使能
 *       5. 舵向轮位置目标输出
 *
 * @important 舵向电机 ID 为 5~8。
 */
void Swerve_Chassis_Update(SwerveChassis* chassis);

#endif /* __SWERVE_CHASSIS_H__ */