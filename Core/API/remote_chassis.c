// #include "remote_chassis.h"
// #include "usart.h"
// #include <math.h>
// #include <stdio.h>
// #include <stdarg.h>
// #include <string.h>

// /* ---------------- 内部结构体 ---------------- */

// typedef struct
// {
//     float max_vx;
//     float max_vy;
//     float max_wz;
// } RemoteSpeedLimit;


// /* ---------------- 串口1调试打印 ---------------- */

// static void Remote_Debug_Printf(const char *format, ...)
// {
//     char buffer[500];
//     va_list args;

//     va_start(args, format);
//     vsnprintf(buffer, sizeof(buffer), format, args);
//     va_end(args);

//     HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
// }


// /**
//  * @brief 把浮点数按 xxx.xxx 格式打印到字符串中，不依赖 printf 浮点支持
//  */
// static void remote_float_to_string(float value, char *str, uint16_t len)
// {
//     int32_t int_part;
//     int32_t frac_part;
//     float abs_value;

//     if (str == NULL || len == 0)
//     {
//         return;
//     }

//     if (value < 0.0f)
//     {
//         abs_value = -value;
//         int_part = (int32_t)abs_value;
//         frac_part = (int32_t)((abs_value - (float)int_part) * 1000.0f + 0.5f);

//         if (frac_part >= 1000)
//         {
//             int_part++;
//             frac_part -= 1000;
//         }

//         snprintf(str, len, "-%ld.%03ld", (long)int_part, (long)frac_part);
//     }
//     else
//     {
//         abs_value = value;
//         int_part = (int32_t)abs_value;
//         frac_part = (int32_t)((abs_value - (float)int_part) * 1000.0f + 0.5f);

//         if (frac_part >= 1000)
//         {
//             int_part++;
//             frac_part -= 1000;
//         }

//         snprintf(str, len, "%ld.%03ld", (long)int_part, (long)frac_part);
//     }
// }


// /**
//  * @brief 判断三挡开关是否接近目标挡位
//  */
// static uint8_t remote_switch_is(uint16_t value, uint16_t target)
// {
//     if (value > target)
//     {
//         return ((value - target) <= REMOTE_SW_TOLERANCE);
//     }
//     else
//     {
//         return ((target - value) <= REMOTE_SW_TOLERANCE);
//     }
// }


// /**
//  * @brief 浮点限幅
//  */
// static float remote_limit_float(float value, float min, float max)
// {
//     if (value > max)
//     {
//         return max;
//     }

//     if (value < min)
//     {
//         return min;
//     }

//     return value;
// }


// /**
//  * @brief 将 iBUS 原始通道值归一化到 [-1, 1]
//  * @param value iBUS 原始通道值，通常 1000~2000，中值 1500
//  *
//  * 例如：
//  * value = 1500 -> 0
//  * value = 2000 -> 1
//  * value = 1000 -> -1
//  */
// static float remote_channel_to_norm(uint16_t value)
// {
//     int32_t diff;

//     if (value < 900u || value > 2100u)
//     {
//         return 0.0f;
//     }

//     diff = (int32_t)value - (int32_t)REMOTE_CENTER;

//     if (diff < 0)
//     {
//         if ((uint32_t)(-diff) <= REMOTE_DEADBAND)
//         {
//             return 0.0f;
//         }
//     }
//     else
//     {
//         if ((uint32_t)diff <= REMOTE_DEADBAND)
//         {
//             return 0.0f;
//         }
//     }

//     return remote_limit_float((float)diff / REMOTE_SPAN, -1.0f, 1.0f);
// }


// /**
//  * @brief 线性映射函数：遥控器原始数字量 -> 速度量
//  *
//  * @param value       遥控器通道原始值，通常 1000~2000
//  * @param max_output  当前挡位的最大速度限幅
//  * @param coef        线性映射系数
//  * @param final_limit 最终硬限幅
//  *
//  * @return 映射后的速度
//  *
//  * 计算逻辑：
//  * 1. value 减去中值 1500
//  * 2. 经过死区
//  * 3. 除以 500，归一化到 -1~1
//  * 4. 乘以 max_output
//  * 5. 再乘以 coef
//  * 6. 先限制在当前挡位速度范围内
//  * 7. 再限制在最终硬限幅范围内
//  */
// static float remote_channel_to_speed(
//     uint16_t value,
//     float max_output,
//     float coef,
//     float final_limit
// )
// {
//     float norm;
//     float speed;

//     norm = remote_channel_to_norm(value);

//     speed = norm * max_output * coef;

//     /*
//      * 第一层限幅：不超过当前挡位速度上限
//      */
//     speed = remote_limit_float(
//         speed,
//         -max_output,
//         max_output
//     );

//     /*
//      * 第二层限幅：最终硬限幅
//      * 当前设置下，传给底盘的最大值为 ±20。
//      */
//     speed = remote_limit_float(
//         speed,
//         -final_limit,
//         final_limit
//     );

//     return speed;
// }


// /**
//  * @brief 根据 SWB 三挡开关选择速度挡位
//  */
// static RemoteSpeedLimit remote_get_speed_limit(uint16_t swb)
// {
//     RemoteSpeedLimit limit;

//     if (remote_switch_is(swb, REMOTE_SW_LOW))
//     {
//         limit.max_vx = REMOTE_FAST_MAX_VX;
//         limit.max_vy = REMOTE_FAST_MAX_VY;
//         limit.max_wz = REMOTE_FAST_MAX_WZ;
//     }
//     else if (remote_switch_is(swb, REMOTE_SW_HIGH))
//     {
//         limit.max_vx = REMOTE_SLOW_MAX_VX;
//         limit.max_vy = REMOTE_SLOW_MAX_VY;
//         limit.max_wz = REMOTE_SLOW_MAX_WZ;
//     }
//     else
//     {
//         limit.max_vx = REMOTE_MID_MAX_VX;
//         limit.max_vy = REMOTE_MID_MAX_VY;
//         limit.max_wz = REMOTE_MID_MAX_WZ;
//     }

//     return limit;
// }


// /**
//  * @brief 遥控器控制底盘
//  */
// void Remote_Chassis_Update(SwerveChassis* chassis)
// {
//     FsIa10bData rc_data;
//     RemoteSpeedLimit speed_limit;

//     float vx_norm;
//     float vy_norm;
//     float wz_norm;

//     float vx;
//     float vy;
//     float wz;

//     char vx_str[20];
//     char vy_str[20];
//     char wz_str[20];

//     char limit_vx_str[20];
//     char limit_vy_str[20];
//     char limit_wz_str[20];

//     static uint32_t last_debug_tick = 0;
//     uint32_t now_tick;

//     if (chassis == NULL)
//     {
//         return;
//     }

//     ibus_maintain();

//     now_tick = HAL_GetTick();

//     /*
//      * 离线保护：
//      * 如果 UART5 没有收到有效 iBUS 帧，这里会一直停车。
//      */
//     if (!ibus_get_data(&rc_data) || !ibus_is_online(REMOTE_TIMEOUT_MS))
//     {
//         Swerve_Chassis_Set_Velocity(
//             chassis,
//             0.0f,
//             0.0f,
//             0.0f
//         );

//         if (now_tick - last_debug_tick >= REMOTE_DEBUG_PERIOD_MS)
//         {
//             last_debug_tick = now_tick;

//             Remote_Debug_Printf(
//                 "IBUS OFFLINE | frame=%lu err=%lu\r\n",
//                 (unsigned long)rc_data.frame_count,
//                 (unsigned long)rc_data.error_count
//             );
//         }

//         return;
//     }

//     /*
//      * VRB 使能保护：
//      * 只有 VRB 被拧到低位，也就是 VRB <= 1200 时，才允许遥控底盘。
//      */
//     if (rc_data.channel[REMOTE_CH_VRB] > REMOTE_VRB_ENABLE_THRESHOLD)
//     {
//         Swerve_Chassis_Set_Velocity(
//             chassis,
//             0.0f,
//             0.0f,
//             0.0f
//         );

//         if (now_tick - last_debug_tick >= REMOTE_DEBUG_PERIOD_MS)
//         {
//             last_debug_tick = now_tick;

//             Remote_Debug_Printf(
//                 "VRB_LOCK | VRB=%u | CH1=%u CH2=%u CH4=%u | frame=%lu err=%lu\r\n",
//                 rc_data.channel[REMOTE_CH_VRB],
//                 rc_data.channel[REMOTE_CH_RIGHT_X],
//                 rc_data.channel[REMOTE_CH_RIGHT_Y],
//                 rc_data.channel[REMOTE_CH_LEFT_X],
//                 (unsigned long)rc_data.frame_count,
//                 (unsigned long)rc_data.error_count
//             );
//         }

//         return;
//     }

//     /*
//      * 根据 SWB 选择速度挡位。
//      */
//     speed_limit = remote_get_speed_limit(rc_data.channel[REMOTE_CH_SWB]);

//     /*
//      * 归一化值只用于调试打印。
//      */
//     vx_norm = remote_channel_to_norm(rc_data.channel[REMOTE_CH_RIGHT_Y]);
//     vy_norm = remote_channel_to_norm(rc_data.channel[REMOTE_CH_RIGHT_X]);
//     wz_norm = remote_channel_to_norm(rc_data.channel[REMOTE_CH_LEFT_X]);

//     /*
//      * 线性映射：
//      *
//      * 原始通道值 -> 归一化 -> 挡位限幅 -> 系数 -> 最终硬限幅
//      *
//      * CH2/index1：右摇杆纵向 -> vx
//      * CH1/index0：右摇杆横向 -> vy
//      * CH4/index3：左摇杆横向 -> wz
//      *
//      * 如果方向反了，只改前面的负号。
//      */
//     vx = -remote_channel_to_speed(
//         rc_data.channel[REMOTE_CH_RIGHT_Y],
//         speed_limit.max_vx,
//         REMOTE_VX_COEF,
//         REMOTE_FINAL_LIMIT_VX
//     );

//     vy = -remote_channel_to_speed(
//         rc_data.channel[REMOTE_CH_RIGHT_X],
//         speed_limit.max_vy,
//         REMOTE_VY_COEF,
//         REMOTE_FINAL_LIMIT_VY
//     );

//     wz = -remote_channel_to_speed(
//         rc_data.channel[REMOTE_CH_LEFT_X],
//         speed_limit.max_wz,
//         REMOTE_WZ_COEF,
//         REMOTE_FINAL_LIMIT_WZ
//     );

//     /*
//      * 这里才是真正传给底盘的值。
//      * 当前最大为：
//      * vx = ±20.0
//      * vy = ±20.0
//      * wz = ±20.0
//      */
//     Swerve_Chassis_Set_Velocity(
//         chassis,
//         vx,
//         vy,
//         wz
//     );

//     /*
//      * USART1 打印所有关键遥控值。
//      * 这里打印的 OUT 就是真正传给底盘的值，不再乘 1000。
//      */
//     if (now_tick - last_debug_tick >= REMOTE_DEBUG_PERIOD_MS)
//     {
//         last_debug_tick = now_tick;

//         remote_float_to_string(vx, vx_str, sizeof(vx_str));
//         remote_float_to_string(vy, vy_str, sizeof(vy_str));
//         remote_float_to_string(wz, wz_str, sizeof(wz_str));

//         remote_float_to_string(REMOTE_FINAL_LIMIT_VX, limit_vx_str, sizeof(limit_vx_str));
//         remote_float_to_string(REMOTE_FINAL_LIMIT_VY, limit_vy_str, sizeof(limit_vy_str));
//         remote_float_to_string(REMOTE_FINAL_LIMIT_WZ, limit_wz_str, sizeof(limit_wz_str));

//         Remote_Debug_Printf(
//             "VRB_ENABLE | VRB=%u | "
//             "CH1=%u CH2=%u CH3=%u CH4=%u CH5=%u CH6=%u CH7=%u CH8=%u CH9=%u CH10=%u | "
//             "N: vx=%d vy=%d wz=%d | "
//             "K: vx=%d vy=%d wz=%d | "
//             "OUT: vx=%s vy=%s wz=%s | "
//             "LIMIT: vx=%s vy=%s wz=%s | "
//             "frame=%lu err=%lu\r\n",

//             rc_data.channel[REMOTE_CH_VRB],

//             rc_data.channel[0],
//             rc_data.channel[1],
//             rc_data.channel[2],
//             rc_data.channel[3],
//             rc_data.channel[4],
//             rc_data.channel[5],
//             rc_data.channel[6],
//             rc_data.channel[7],
//             rc_data.channel[8],
//             rc_data.channel[9],

//             (int)(vx_norm * 1000.0f),
//             (int)(vy_norm * 1000.0f),
//             (int)(wz_norm * 1000.0f),

//             (int)(REMOTE_VX_COEF * 1000.0f),
//             (int)(REMOTE_VY_COEF * 1000.0f),
//             (int)(REMOTE_WZ_COEF * 1000.0f),

//             vx_str,
//             vy_str,
//             wz_str,

//             limit_vx_str,
//             limit_vy_str,
//             limit_wz_str,

//             (unsigned long)rc_data.frame_count,
//             (unsigned long)rc_data.error_count
//         );
//     }
// }
#include "remote_chassis.h"

/* ---------------- 内部结构体 ---------------- */

typedef struct
{
    float max_vx;
    float max_vy;
    float max_wz;
} RemoteSpeedLimit;


/**
 * @brief 判断三挡开关是否接近目标挡位
 */
static uint8_t remote_switch_is(uint16_t value, uint16_t target)
{
    if (value > target)
    {
        return ((value - target) <= REMOTE_SW_TOLERANCE);
    }
    else
    {
        return ((target - value) <= REMOTE_SW_TOLERANCE);
    }
}


/**
 * @brief 浮点限幅
 */
static float remote_limit_float(float value, float min, float max)
{
    if (value > max)
    {
        return max;
    }

    if (value < min)
    {
        return min;
    }

    return value;
}


/**
 * @brief 将 iBUS 原始通道值归一化到 [-1, 1]
 *
 * value = 1500 -> 0
 * value = 2000 -> 1
 * value = 1000 -> -1
 */
static float remote_channel_to_norm(uint16_t value)
{
    int32_t diff;

    /*
     * 防止异常通道值导致误动作。
     */
    if (value < 900u || value > 2100u)
    {
        return 0.0f;
    }

    diff = (int32_t)value - (int32_t)REMOTE_CENTER;

    /*
     * 死区处理。
     */
    if (diff < 0)
    {
        if ((uint32_t)(-diff) <= REMOTE_DEADBAND)
        {
            return 0.0f;
        }
    }
    else
    {
        if ((uint32_t)diff <= REMOTE_DEADBAND)
        {
            return 0.0f;
        }
    }

    return remote_limit_float((float)diff / REMOTE_SPAN, -1.0f, 1.0f);
}


/**
 * @brief 线性映射函数：遥控器原始通道值 -> 速度值
 *
 * 实际输出：
 *
 * speed = norm * 当前挡位最大速度 * 系数
 *
 * 然后进行两层限幅：
 *
 * 1. 不超过当前挡位最大速度
 * 2. 不超过最终硬限幅
 */
static float remote_channel_to_speed(
    uint16_t value,
    float max_output,
    float coef,
    float final_limit
)
{
    float norm;
    float speed;

    norm = remote_channel_to_norm(value);

    speed = norm * max_output * coef;

    /*
     * 第一层限幅：当前挡位限幅。
     */
    speed = remote_limit_float(
        speed,
        -max_output,
        max_output
    );

    /*
     * 第二层限幅：最终硬限幅。
     */
    speed = remote_limit_float(
        speed,
        -final_limit,
        final_limit
    );

    return speed;
}


/**
 * @brief 根据 SWB 三挡开关选择速度挡位
 */
static RemoteSpeedLimit remote_get_speed_limit(uint16_t swb)
{
    RemoteSpeedLimit limit;

    if (remote_switch_is(swb, REMOTE_SW_LOW))
    {
        /*
         * SWB = LOW，高速挡。
         */
        limit.max_vx = REMOTE_FAST_MAX_VX;
        limit.max_vy = REMOTE_FAST_MAX_VY;
        limit.max_wz = REMOTE_FAST_MAX_WZ;
    }
    else if (remote_switch_is(swb, REMOTE_SW_HIGH))
    {
        /*
         * SWB = HIGH，低速挡。
         */
        limit.max_vx = REMOTE_SLOW_MAX_VX;
        limit.max_vy = REMOTE_SLOW_MAX_VY;
        limit.max_wz = REMOTE_SLOW_MAX_WZ;
    }
    else
    {
        /*
         * 其他情况默认中速挡。
         */
        limit.max_vx = REMOTE_MID_MAX_VX;
        limit.max_vy = REMOTE_MID_MAX_VY;
        limit.max_wz = REMOTE_MID_MAX_WZ;
    }

    return limit;
}


/**
 * @brief 给底盘发送零速度
 *
 * 注意：
 * 这里只是速度清零，不主动关闭电机，不主动失能舵轮。
 */
static void remote_chassis_set_zero_velocity(SwerveChassis* chassis)
{
    Swerve_Chassis_Set_Velocity(
        chassis,
        0.0f,
        0.0f,
        0.0f
    );
}


/**
 * @brief 遥控器控制底盘
 */
void Remote_Chassis_Update(SwerveChassis* chassis)
{
    FsIa10bData rc_data;
    RemoteSpeedLimit speed_limit;

    float vx;
    float vy;
    float wz;

    if (chassis == NULL)
    {
        return;
    }

    /*
     * 保持 iBUS 接收。
     * 这一步必须一直调用，否则 UART5 接收异常后可能无法恢复。
     */
    ibus_maintain();

    /*
     * 离线保护：
     * 如果没有收到有效 iBUS 帧，只给底盘 0 速度。
     *
     * 注意：
     * 这里不做电机失能，只锁速度。
     */
    if (!ibus_get_data(&rc_data) || !ibus_is_online(REMOTE_TIMEOUT_MS))
    {
        remote_chassis_set_zero_velocity(chassis);
        return;
    }

    /*
     * VRB 速度使能保护：
     *
     * VRB <= 1200：允许遥控器控制速度
     * VRB > 1200 ：只给 0 速度，不主动失能舵轮
     */
    if (rc_data.channel[REMOTE_CH_VRB] > REMOTE_VRB_ENABLE_THRESHOLD)
    {
        remote_chassis_set_zero_velocity(chassis);
        return;
    }

    /*
     * 根据 SWB 选择速度挡位。
     */
    speed_limit = remote_get_speed_limit(rc_data.channel[REMOTE_CH_SWB]);

    /*
     * 通道映射：
     *
     * CH2 / index 1：右摇杆纵向 -> vx
     * CH1 / index 0：右摇杆横向 -> vy
     * CH4 / index 3：左摇杆横向 -> wz
     *
     * 前面的负号用于调整方向。
     * 如果方向反了，只改对应变量前面的负号。
     */
    vx = -remote_channel_to_speed(
        rc_data.channel[REMOTE_CH_RIGHT_Y],
        speed_limit.max_vx,
        REMOTE_VX_COEF,
        REMOTE_FINAL_LIMIT_VX
    );

    vy = -remote_channel_to_speed(
        rc_data.channel[REMOTE_CH_RIGHT_X],
        speed_limit.max_vy,
        REMOTE_VY_COEF,
        REMOTE_FINAL_LIMIT_VY
    );

    wz = -remote_channel_to_speed(
        rc_data.channel[REMOTE_CH_LEFT_X],
        speed_limit.max_wz,
        REMOTE_WZ_COEF,
        REMOTE_FINAL_LIMIT_WZ
    );

    /*
     * 真正传给底盘解算的值。
     *
     * 高速挡最大：
     * vx = ±20.0
     * vy = ±20.0
     * wz = ±20.0
     *
     * 中速挡最大：
     * vx = ±10.0
     * vy = ±10.0
     * wz = ±10.0
     *
     * 低速挡最大：
     * vx = ±5.0
     * vy = ±5.0
     * wz = ±5.0
     */
    Swerve_Chassis_Set_Velocity(
        chassis,
        vx,
        vy,
        wz
    );
}