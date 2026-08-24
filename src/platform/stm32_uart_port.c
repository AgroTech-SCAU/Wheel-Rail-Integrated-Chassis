#include "stm32_uart_port.h"

#include "usart.h"

#include <stddef.h>

typedef struct {
    Stm32UartEventCallback rx_complete;
    Stm32UartEventCallback error;
    void* context;
} Stm32UartCallbacks;

static Stm32UartCallbacks g_uart5_callbacks;

bool stm32_uart5_start_receive(uint8_t* data, uint16_t len)
{
    return data != NULL && len > 0U &&
           HAL_UART_Receive_IT(&huart5, data, len) == HAL_OK;
}

bool stm32_uart5_abort_receive(void)
{
    return HAL_UART_AbortReceive_IT(&huart5) == HAL_OK;
}

bool stm32_uart5_receive_is_ready(void)
{
    return huart5.RxState == HAL_UART_STATE_READY;
}

bool stm32_uart5_register_callbacks(Stm32UartEventCallback rx_complete,
                                    Stm32UartEventCallback error,
                                    void* context)
{
    if (rx_complete == NULL || error == NULL) {
        return false;
    }
    if (g_uart5_callbacks.rx_complete != NULL &&
        (g_uart5_callbacks.rx_complete != rx_complete ||
         g_uart5_callbacks.error != error ||
         g_uart5_callbacks.context != context)) {
        return false;
    }
    g_uart5_callbacks.rx_complete = rx_complete;
    g_uart5_callbacks.error = error;
    g_uart5_callbacks.context = context;
    return true;
}

bool stm32_usart1_write(const char* data, uint32_t len)
{
    if (data == NULL || len == 0U || len > 0xFFFFU) {
        return false;
    }
    return HAL_UART_Transmit(&huart1, (uint8_t*)data, (uint16_t)len,
                             HAL_MAX_DELAY) == HAL_OK;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* handle)
{
    if (handle != NULL && handle->Instance == UART5 &&
        g_uart5_callbacks.rx_complete != NULL) {
        g_uart5_callbacks.rx_complete(g_uart5_callbacks.context);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* handle)
{
    if (handle != NULL && handle->Instance == UART5 &&
        g_uart5_callbacks.error != NULL) {
        g_uart5_callbacks.error(g_uart5_callbacks.context);
    }
}
