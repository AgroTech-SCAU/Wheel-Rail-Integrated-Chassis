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

    /* ---------------- 【核心修改点】仅在 wz 触发时，调换 ID 5/7 的 wz 方向并多转 90 度 ---------------- */
    if (fabsf(chassis->kine.control.wz) > 1e-6f)
    {
        float hx = chassis->model.length * 0.5f;
        float hy = chassis->model.width * 0.5f;
        float vx = chassis->kine.control.vx;
        float vy = chassis->kine.control.vy;
        float inv_wz = -chassis->kine.control.wz; // 将 wz 取反
        float offset_angle = 1.570796327f;        // 90 度的弧度值 (PI / 2)

        // 1. 重新计算 FL (Index 0, 对应 ID 5): 坐标 x = hx, y = hy
        float vix0 = vx - inv_wz * hy;
        float viy0 = vy + inv_wz * hx;
        float spd0 = sqrtf(vix0 * vix0 + viy0 * viy0);
        chassis->kine.control.wheels[0].wheel_omega = spd0 / chassis->model.wheel_radius;
        if (spd0 > 1e-6f) {
            float angle0 = atan2f(viy0, vix0) + offset_angle;
            // 确保角度范围在 (-PI, PI] 内
            if (angle0 > 3.141592654f) angle0 -= 6.283185307f;
            chassis->kine.control.wheels[0].steer_angle = angle0;
        } else {
            float angle0 = chassis->kine.control.wheels[0].steer_angle + offset_angle;
            if (angle0 > 3.141592654f) angle0 -= 6.283185307f;
            chassis->kine.control.wheels[0].steer_angle = angle0;
        }

        // 2. 重新计算 RR (Index 2, 对应 ID 7): 坐标 x = -hx, y = -hy
        float vix2 = vx - inv_wz * (-hy);
        float viy2 = vy + inv_wz * (-hx);
        float spd2 = sqrtf(vix2 * vix2 + viy2 * viy2);
        chassis->kine.control.wheels[2].wheel_omega = spd2 / chassis->model.wheel_radius;
        if (spd2 > 1e-6f) {
            float angle2 = atan2f(viy2, vix2) + offset_angle;
            // 确保角度范围在 (-PI, PI] 内
            if (angle2 > 3.141592654f) angle2 -= 6.283185307f;
            chassis->kine.control.wheels[2].steer_angle = angle2;
        } else {
            float angle2 = chassis->kine.control.wheels[2].steer_angle + offset_angle;
            if (angle2 > 3.141592654f) angle2 -= 6.283185307f;
            chassis->kine.control.wheels[2].steer_angle = angle2;
        }
    }

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

        Motor_Speed_Control_Smooth(
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