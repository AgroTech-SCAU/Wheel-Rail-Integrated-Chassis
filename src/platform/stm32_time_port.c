/**
 * @file stm32_time_port.c
 * @brief STM32 时间与临界区适配实现
 */

#include "stm32_time_port.h"

#include "stm32h7xx_hal.h"

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 获取 HAL 毫秒时间戳
 * @return uint32_t 当前毫秒计数
 */
uint32_t stm32_time_now_ms(void) {
    return HAL_GetTick();
}

/**
 * @brief 执行阻塞毫秒延时
 * @param delay_ms 延时时间 单位 ms
 */
void stm32_time_delay_ms(uint32_t delay_ms) {
    HAL_Delay(delay_ms);
}

/**
 * @brief 进入全局临界区
 * @return uint32_t 进入临界区前的 PRIMASK 状态
 */
uint32_t stm32_critical_enter(void) {
    uint32_t state = __get_PRIMASK();
    __disable_irq();
    return state;
}

/**
 * @brief 恢复进入临界区前的中断状态
 * @param state 进入临界区前的 PRIMASK 状态
 */
void stm32_critical_exit(uint32_t state) {
    if(state == 0u) {
        __enable_irq();
    }
}
