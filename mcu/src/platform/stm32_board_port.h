#ifndef _stm32_board_port_h_
#define _stm32_board_port_h_

/**
 * @file stm32_board_port.h
 * @brief STM32 底盘板级控制适配接口
 */

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool estop_valid;
    bool physical_estop;
    bool battery_valid;
    float battery_voltage;
    float current;
} Stm32BoardInputs;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 使能 PC15 遥控接收器 5V 供电控制信号
 */
void stm32_board_receiver_power_enable(void);

/** Disable PC15 remote receiver 5V supply. This is not a motor power control. */
void stm32_board_receiver_power_disable(void);

/** Hardware RNG boot identifier, generated once per boot; false if RNG unavailable. */
bool stm32_board_boot_id(uint32_t* out);

/** No board wiring is assumed: both validity flags remain false until a producer submits measurements. */
Stm32BoardInputs stm32_board_read_inputs(void);
void stm32_board_submit_inputs(const Stm32BoardInputs* inputs);

#endif
