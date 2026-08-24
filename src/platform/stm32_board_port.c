#include "stm32_board_port.h"

#include "main.h"

void stm32_board_chassis_power_enable(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
}
