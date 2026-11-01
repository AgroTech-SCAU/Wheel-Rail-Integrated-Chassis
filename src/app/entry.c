/**
 * @file entry.c
 * @brief CubeMX 应用入口实现
 */

#include "entry.h"

#include "main.h" // IWYU pragma: keep

#include "chassis_service.h"

// ! ========================= 变 量 声 明 ========================= ! //

static const ChassisServiceConfig s_chassis_config = {
    .model = {
        .length = 0.725f,
        .width = 0.730f,
        .wheel_radius = 0.0215f,
        .max_wheel_linear_speed = 0.45f,
    },
    /* 平移偏航前馈补偿：wz + k_vx * vx_有效 + k_vy * vy_有效。 */
    /* 参考系数需按本车标定，步骤见 docs/yaw_bias_tuning.md。 */
    .yaw_bias = {
        .enabled = true,       /* false 可关闭补偿 */
        .k_vx = -0.065f,       /* x 轴有符号补偿系数，单位 rad/m */
        .k_vy = 0.0466f,        /* y 轴有符号补偿系数，单位 rad/m */
        .v_deadband = 0.01f,   /* 补偿用指令速度死区，单位 m/s */
    },
};

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化底盘应用
 */
void entry_init(void) {
    HAL_Delay(1000);

    (void)chassis_service_init_with_config(&s_chassis_config);
}

/**
 * @brief 执行一次底盘应用轮询
 */
void entry_loop(void) {
    (void)chassis_service_update();
}
