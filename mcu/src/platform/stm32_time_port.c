/**
 * @file stm32_time_port.c
 * @brief STM32 时间与临界区适配实现
 */

#include "stm32_time_port.h"

#include "stm32h7xx_hal.h"
#include "tim.h"

static uint32_t s_last_tim5;
static uint64_t s_elapsed_us;
static bool s_tim5_started;

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 获取 HAL 毫秒时间戳
 * @return uint32_t 当前毫秒计数
 */
uint32_t stm32_time_now_ms(void) {
    return HAL_GetTick();
}

bool stm32_time_init(void) {
    RCC_ClkInitTypeDef clocks = {0};
    uint32_t flash_latency = 0u;
    uint32_t timer_hz;
    if(s_tim5_started) {
        return true;
    }
    if(htim5.Instance != TIM5 || htim5.Init.Period != UINT32_MAX) {
        return false;
    }
    HAL_RCC_GetClockConfig(&clocks, &flash_latency);
    const uint32_t pclk1_hz = HAL_RCC_GetPCLK1Freq();
    /* DIV1 and DIV2 are unambiguous for TIMPRE; reject other clock trees. */
    if(clocks.APB1CLKDivider == RCC_APB1_DIV1) {
        timer_hz = pclk1_hz;
    } else if(clocks.APB1CLKDivider == RCC_APB1_DIV2 &&
              pclk1_hz <= UINT32_MAX / 2u) {
        timer_hz = pclk1_hz * 2u;
    } else {
        return false;
    }
    if(timer_hz < 1000000u || timer_hz % 1000000u != 0u) {
        return false;
    }
    htim5.Init.Prescaler = timer_hz / 1000000u - 1u;
    if(HAL_TIM_Base_Init(&htim5) != HAL_OK ||
       HAL_TIM_Base_Start(&htim5) != HAL_OK) {
        return false;
    }
    const uint32_t state = stm32_critical_enter();
    s_last_tim5 = __HAL_TIM_GET_COUNTER(&htim5);
    s_elapsed_us = 0u;
    s_tim5_started = true;
    stm32_critical_exit(state);
    return true;
}

uint64_t stm32_time_now_us(void) {
    const uint32_t state = stm32_critical_enter();
    if(!s_tim5_started) {
        stm32_critical_exit(state);
        return 0u;
    }
    const uint32_t now = __HAL_TIM_GET_COUNTER(&htim5);
    s_elapsed_us += (uint32_t)(now - s_last_tim5);
    s_last_tim5 = now;
    const uint64_t elapsed = s_elapsed_us;
    stm32_critical_exit(state);
    return elapsed;
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
