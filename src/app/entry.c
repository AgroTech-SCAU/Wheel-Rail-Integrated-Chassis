/**
 * @file entry.c
 * @brief CubeMX 应用入口实现
 */

#include "entry.h"

#include "chassis_service.h"

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化底盘应用
 */
void entry_init(void) {
    (void)chassis_service_init();
}

/**
 * @brief 执行一次底盘应用轮询
 */
void entry_loop(void) {
    (void)chassis_service_update();
}
