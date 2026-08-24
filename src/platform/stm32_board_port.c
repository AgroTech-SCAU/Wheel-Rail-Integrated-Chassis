/**
 * @file stm32_board_port.c
 * @brief STM32 底盘板级控制适配实现
 */

#include "stm32_board_port.h"

#include "main.h"

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 使能底盘执行器供电控制信号
 */
void stm32_board_chassis_power_enable(void) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
}
