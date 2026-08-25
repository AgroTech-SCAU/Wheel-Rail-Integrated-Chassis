#ifndef _stm32_board_port_h_
#define _stm32_board_port_h_

/**
 * @file stm32_board_port.h
 * @brief STM32 底盘板级控制适配接口
 */

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 使能底盘执行器供电控制信号
 */
void stm32_board_chassis_power_enable(void);

#endif
