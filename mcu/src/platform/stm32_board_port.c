/**
 * @file stm32_board_port.c
 * @brief STM32 底盘板级控制适配实现
 */

#include "stm32_board_port.h"

#include "main.h"
#include "rng.h"
#include "stm32_time_port.h"

#include <math.h>
#include <stddef.h>

static Stm32BoardInputs s_inputs;
static uint32_t s_boot_id;

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 使能 PC15 遥控接收器 5V 供电控制信号
 */
void stm32_board_receiver_power_enable(void) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
}

void stm32_board_receiver_power_disable(void) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
}

bool stm32_board_boot_id(uint32_t* out) {
    if(out == NULL || hrng.Instance != RNG) {
        return false;
    }
    if(s_boot_id == 0u) {
        uint32_t generated = 0u;
        for(unsigned attempt = 0u; attempt < 4u; ++attempt) {
            if(HAL_RNG_GenerateRandomNumber(&hrng, &generated) != HAL_OK) {
                return false;
            }
            if(generated != 0u) {
                s_boot_id = generated;
                break;
            }
        }
    }
    *out = s_boot_id;
    return s_boot_id != 0u;
}

Stm32BoardInputs stm32_board_read_inputs(void) {
    const uint32_t state = stm32_critical_enter();
    const Stm32BoardInputs result = s_inputs;
    stm32_critical_exit(state);
    return result;
}

void stm32_board_submit_inputs(const Stm32BoardInputs* inputs) {
    if(inputs == NULL) {
        return;
    }
    Stm32BoardInputs value = *inputs;
    if(value.battery_valid && (!isfinite(value.battery_voltage) ||
                               !isfinite(value.current))) {
        value.battery_valid = false;
    }
    const uint32_t state = stm32_critical_enter();
    s_inputs = value;
    stm32_critical_exit(state);
}
