#include "swerve_chassis.h"
#include <math.h>

/* ---------------- 宏定义 ---------------- */

#define RADPS_TO_RPM 9.5492965855f

extern FDCAN_HandleTypeDef hfdcan2;

/**
 * @brief 初始化底盘物理参数
 */
void Swerve_Chassis_Model_Init(
    SwerveChassis* chassis,
    float length,
    float width,
    float wheel_radius,
    float max_speed
)
{
    if(chassis == NULL) return;

    chassis->model.length = length;
    chassis->model.width = width;
    chassis->model.wheel_radius = wheel_radius;
    chassis->model.max_wheel_linear_speed = max_speed;
}

/**
 * @brief 初始化舵轮底盘
 */
void Swerve_Chassis_Init(SwerveChassis* chassis)
{
    if(chassis == NULL) return;

    /* ---------------- 初始化运动学模型 ---------------- */

    swheel.init(
        &chassis->kine,
        chassis->model
    );

    /* ---------------- 驱动轮 ID ---------------- */

    chassis->drive_ids[0] = 1; // FL
    chassis->drive_ids[1] = 2; // FR
    chassis->drive_ids[2] = 3; // RR
    chassis->drive_ids[3] = 4; // RL

    /* ---------------- 舵向轮 ID ---------------- */

    chassis->steer_ids[0] = 5; // FL
    chassis->steer_ids[1] = 6; // FR
    chassis->steer_ids[2] = 7; // RR
    chassis->steer_ids[3] = 8; // RL

    /* ---------------- 初始化驱动轮 ---------------- */

    Motor_Driver_Init();

    /* ---------------- 初始化舵向轮 ---------------- */

    for(int i = 0; i < 4; i++)
    {
        RS06_Set_Mode(
            &hfdcan2,
            chassis->steer_ids[i],
            RS06_MODE_PP
        );

        HAL_Delay(5);

        RS06_Enable(
            &hfdcan2,
            chassis->steer_ids[i]
        );

        HAL_Delay(5);
    }
}

/**
 * @brief 设置底盘速度
 */
void Swerve_Chassis_Set_Velocity(
    SwerveChassis* chassis,
    float vx,
    float vy,
    float wz
)
{
    if(chassis == NULL) return;

    chassis->kine.control.vx = vx;
    chassis->kine.control.vy = vy;
    chassis->kine.control.wz = wz;
}

/**
 * @brief 更新舵轮底盘
 */
void Swerve_Chassis_Update(SwerveChassis* chassis)
{
    if(chassis == NULL) return;

    /* ---------------- IK 解算 ---------------- */

    swheel.ik(&chassis->kine);

    /* ---------------- 输出到真实电机 ---------------- */

    for(int i = 0; i < 4; i++)
    {
        /* ============================================
           1. 驱动轮速度控制
           ============================================ */

        float omega_rad_s =
            chassis->kine.control.wheels[i].wheel_omega;

        int16_t rpm =
            (int16_t)(omega_rad_s * RADPS_TO_RPM);

        Motor_Speed_Control(
            rpm,
            chassis->drive_ids[i]
        );

        /* ============================================
           2. 舵向轮角度控制
           ============================================ */

        float angle_rad =
            chassis->kine.control.wheels[i].steer_angle;

        RS06_Set_Position_Target(
            &hfdcan2,
            chassis->steer_ids[i],
            angle_rad
        );
    }
}