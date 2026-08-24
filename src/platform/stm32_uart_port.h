#ifndef STM32_UART_PORT_H
#define STM32_UART_PORT_H

#include <stdbool.h>
#include <stdint.h>

typedef void (*Stm32UartEventCallback)(void* context);

bool stm32_uart5_start_receive(uint8_t* data, uint16_t len);
bool stm32_uart5_abort_receive(void);
bool stm32_uart5_receive_is_ready(void);
bool stm32_uart5_register_callbacks(Stm32UartEventCallback rx_complete,
                                    Stm32UartEventCallback error,
                                    void* context);
bool stm32_usart1_write(const char* data, uint32_t len);

#endif /* STM32_UART_PORT_H */
