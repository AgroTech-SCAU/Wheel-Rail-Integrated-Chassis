#ifndef _stm32_time_port_h_
#define _stm32_time_port_h_

/**
 * @file stm32_time_port.h
 * @brief STM32 时间与临界区适配接口
 */

#include <stdint.h>
#include <stdbool.h>

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 获取 HAL 毫秒时间戳
 * @return uint32_t 当前毫秒计数
 */
uint32_t stm32_time_now_ms(void);

/** Start the dedicated 1 MHz TIM5 monotonic clock; failure is fatal to host protocol. */
bool stm32_time_init(void);

/** Monotonic microseconds since TIM5 start. Must be sampled within each 32-bit wrap (~71 min). */
uint64_t stm32_time_now_us(void);

/**
 * @brief 执行阻塞毫秒延时
 * @param delay_ms 延时时间 单位 ms
 */
void stm32_time_delay_ms(uint32_t delay_ms);

/**
 * @brief 进入全局临界区
 * @return uint32_t 进入临界区前的 PRIMASK 状态
 */
uint32_t stm32_critical_enter(void);

/**
 * @brief 恢复进入临界区前的中断状态
 * @param state 进入临界区前的 PRIMASK 状态
 */
void stm32_critical_exit(uint32_t state);

#endif
