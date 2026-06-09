#ifndef __REMOTE_CHASSIS_H__
#define __REMOTE_CHASSIS_H__

#include "swerve_chassis.h"
#include "FS-IA10B.h"
#include <stdint.h>
#include <stdbool.h>

/* ---------------- 通道索引定义 ----------------
 *
 * FS-iA10B iBUS 常见通道映射：
 *
 * CH1 -> index 0：右摇杆横向
 * CH2 -> index 1：右摇杆纵向
 * CH3 -> index 2：通常是油门/辅助
 * CH4 -> index 3：左摇杆横向
 *
 * SWA -> index 4
 * SWB -> index 5，用于速度挡位
 * SWC -> index 6
 * SWD -> index 7
 * VRA -> index 8
 * VRB -> index 9，用于底盘速度使能
 */
#define REMOTE_CH_RIGHT_X      0u
#define REMOTE_CH_RIGHT_Y      1u
#define REMOTE_CH_LEFT_X       3u

#define REMOTE_CH_SWA          4u
#define REMOTE_CH_SWB          5u
#define REMOTE_CH_SWC          6u
#define REMOTE_CH_SWD          7u
#define REMOTE_CH_VRA          8u
#define REMOTE_CH_VRB          9u

/* ---------------- iBUS 原始值参数 ---------------- */

#define REMOTE_CENTER          1500u
#define REMOTE_SPAN            500.0f

/*
 * 摇杆死区。
 */
#define REMOTE_DEADBAND        30u

/*
 * 遥控器离线判定时间，单位 ms。
 */
#define REMOTE_TIMEOUT_MS      200u

/* ---------------- 三挡开关原始值 ---------------- */

#define REMOTE_SW_LOW          2000u
#define REMOTE_SW_CENTER       1500u
#define REMOTE_SW_HIGH         1000u

/*
 * 开关判断容差。
 */
#define REMOTE_SW_TOLERANCE    250u

/* ---------------- VRB 速度使能阈值 ----------------
 *
 * VRB <= 1200 时允许遥控底盘。
 * VRB > 1200 时只给底盘 0 速度，不主动失能电机。
 */
#define REMOTE_VRB_ENABLE_THRESHOLD 1200u

/* ---------------- 三挡速度限幅 ----------------
 *
 * SWB = LOW    ：高速挡，最大 ±20
 * SWB = CENTER ：中速挡，最大 ±10
 * SWB = HIGH   ：低速挡，最大 ±5
 */

/* SWB = LOW，高速挡 */
#define REMOTE_FAST_MAX_VX     20.0f
#define REMOTE_FAST_MAX_VY     20.0f
#define REMOTE_FAST_MAX_WZ     20.0f

/* SWB = CENTER，中速挡 */
#define REMOTE_MID_MAX_VX      10.0f
#define REMOTE_MID_MAX_VY      10.0f
#define REMOTE_MID_MAX_WZ      10.0f

/* SWB = HIGH，低速挡 */
#define REMOTE_SLOW_MAX_VX     5.0f
#define REMOTE_SLOW_MAX_VY     5.0f
#define REMOTE_SLOW_MAX_WZ     5.0f

/* ---------------- 线性映射系数 ---------------- */

#define REMOTE_VX_COEF         1.00f
#define REMOTE_VY_COEF         1.00f
#define REMOTE_WZ_COEF         1.00f

/* ---------------- 最终输出硬限幅 ----------------
 *
 * 最终传给 Swerve_Chassis_Set_Velocity() 的值不会超过 ±20。
 */
#define REMOTE_FINAL_LIMIT_VX  20.0f
#define REMOTE_FINAL_LIMIT_VY  20.0f
#define REMOTE_FINAL_LIMIT_WZ  20.0f

/* ---------------- 接口函数 ---------------- */

void Remote_Chassis_Update(SwerveChassis* chassis);

#endif /* __REMOTE_CHASSIS_H__ */