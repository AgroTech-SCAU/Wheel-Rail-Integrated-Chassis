#ifndef STM32_TIME_PORT_H
#define STM32_TIME_PORT_H

#include <stdint.h>

uint32_t stm32_time_now_ms(void);
void stm32_time_delay_ms(uint32_t delay_ms);
uint32_t stm32_critical_enter(void);
void stm32_critical_exit(uint32_t state);

#endif /* STM32_TIME_PORT_H */
