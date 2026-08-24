#include "stm32_time_port.h"

#include "stm32h7xx_hal.h"

uint32_t stm32_time_now_ms(void)
{
    return HAL_GetTick();
}

void stm32_time_delay_ms(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

uint32_t stm32_critical_enter(void)
{
    uint32_t state = __get_PRIMASK();
    __disable_irq();
    return state;
}

void stm32_critical_exit(uint32_t state)
{
    if (state == 0U) {
        __enable_irq();
    }
}
