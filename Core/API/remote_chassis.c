#include "remote_chassis.h"
#include <math.h>

/**
 * @brief 通道映射函数
 */
static float channel_map(
    uint16_t value,
    float max_output
)
{
    float offset =
        (float)value - IBUS_MID_VALUE;

    /* 死区 */
    if(fabsf(offset) < IBUS_DEADZONE)
    {
        return 0.0f;
    }

    /* 归一化 */
    float normalized =
        offset / IBUS_MAX_OFFSET;

    /* 限幅 */
    if(normalized > 1.0f)
    {
        normalized = 1.0f;
    }

    if(normalized < -1.0f)
    {
        normalized = -1.0f;
    }

    return normalized * max_output;
}

/**
 * @brief 遥控器控制底盘
 */
void Remote_Chassis_Update(
    SwerveChassis* chassis
)
{
    if(chassis == NULL)
    {
        return;
    }

    /* 遥控器离线保护 */

    if(!ibus_is_online(100))
    {
        Swerve_Chassis_Set_Velocity(
            chassis,
            0.0f,
            0.0f,
            0.0f
        );

        return;
    }

    /* ---------------- 读取通道 ---------------- */

    uint16_t ch1 = ibus_get_channel(0); // CH1
    uint16_t ch2 = ibus_get_channel(1); // CH2
    uint16_t ch4 = ibus_get_channel(3); // CH4

    /* ---------------- 通道映射 ---------------- */

    float vx =
        -channel_map(
            ch2,
            CHASSIS_MAX_VX
        );

    float vy =
        channel_map(
            ch1,
            CHASSIS_MAX_VY
        );

    float wz =
        channel_map(
            ch4,
            CHASSIS_MAX_WZ
        );

    /* ---------------- 设置底盘速度 ---------------- */

    Swerve_Chassis_Set_Velocity(
        chassis,
        vx,
        vy,
        wz
    );
}