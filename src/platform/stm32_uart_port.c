/**
 * @file stm32_uart_port.c
 * @brief STM32 UART 接收与日志输出适配实现
 */

#include "stm32_uart_port.h"

#include "usart.h"

#include <stddef.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief UART5 事件回调注册槽位
 */
typedef struct {
    Stm32UartEventCallback rx_complete;
    Stm32UartEventCallback error;
    void* context;
} Stm32UartCallbacks;

static Stm32UartCallbacks s_uart5_callbacks;

// ! ========================= 接 口 函 数 实 现 ========================= ! //

bool stm32_uart5_start_receive(uint8_t* data, uint16_t len) {
    return data != NULL && len > 0u &&
           HAL_UART_Receive_IT(&huart5, data, len) == HAL_OK;
}

bool stm32_uart5_abort_receive(void) {
    return HAL_UART_AbortReceive_IT(&huart5) == HAL_OK;
}

bool stm32_uart5_receive_is_ready(void) {
    return huart5.RxState == HAL_UART_STATE_READY;
}

bool stm32_uart5_register_callbacks(Stm32UartEventCallback rx_complete,
                                    Stm32UartEventCallback error,
                                    void* context) {
    if(rx_complete == NULL || error == NULL) {
        return false;
    }
    if(s_uart5_callbacks.rx_complete != NULL &&
       (s_uart5_callbacks.rx_complete != rx_complete ||
        s_uart5_callbacks.error != error ||
        s_uart5_callbacks.context != context)) {
        return false;
    }
    s_uart5_callbacks.rx_complete = rx_complete;
    s_uart5_callbacks.error = error;
    s_uart5_callbacks.context = context;
    return true;
}

bool stm32_usart1_write(const char* data, uint32_t len) {
    if(data == NULL || len == 0u || len > 0xFFFFu) {
        return false;
    }
    return HAL_UART_Transmit(&huart1, (uint8_t*)data, (uint16_t)len,
                             HAL_MAX_DELAY) == HAL_OK;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* handle) {
    if(handle != NULL && handle->Instance == UART5 &&
       s_uart5_callbacks.rx_complete != NULL) {
        s_uart5_callbacks.rx_complete(s_uart5_callbacks.context);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* handle) {
    if(handle != NULL && handle->Instance == UART5 &&
       s_uart5_callbacks.error != NULL) {
        s_uart5_callbacks.error(s_uart5_callbacks.context);
    }
}
