#ifndef __REMOTE_CHASSIS_H__
#define __REMOTE_CHASSIS_H__

#include "swerve_chassis.h"
#include "FS-IA10B.h"

/* ---------------- 参数定义 ---------------- */

/* 遥控器中值 */
#define IBUS_MID_VALUE         1500.0f

/* 死区 */
#define IBUS_DEADZONE          30.0f

/* 摇杆最大偏移 */
#define IBUS_MAX_OFFSET        500.0f

/* 最大底盘速度 */
#define CHASSIS_MAX_VX         2.0f
#define CHASSIS_MAX_VY         2.0f
#define CHASSIS_MAX_WZ         4.0f

/* ---------------- 接口函数 ---------------- */

/**
 * @brief 遥控器控制更新
 */
void Remote_Chassis_Update(SwerveChassis* chassis);

#endif