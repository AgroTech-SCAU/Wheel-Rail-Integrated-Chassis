#include "FS-IA10B.h"
#include <string.h>

extern UART_HandleTypeDef huart5;

static uint8_t s_rx_byte = 0u;
static uint8_t s_frame[FS_IA10B_IBUS_FRAME_LEN];
static uint8_t s_frame_index = 0u;
static volatile FsIa10bData s_data;

void ibus_init(void) {
    memset((void*)&s_data, 0, sizeof(s_data));
    s_frame_index = 0u;
    HAL_UART_Receive_IT(&huart5, &s_rx_byte, 1);
}

void ibus_maintain(void) {
    // 自动重连机制：如果串口被意外关闭，尝试重新开启
    if (huart5.RxState == HAL_UART_STATE_READY) {
        HAL_UART_Receive_IT(&huart5, &s_rx_byte, 1);
    }
}

// 在 main.c 的 HAL_UART_RxCpltCallback 中调用此函数
void ibus_rx_complete_callback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART5) {
        // 状态机解析
        if (s_frame_index == 0u) {
            if (s_rx_byte == 0x20) { s_frame[0] = 0x20; s_frame_index = 1; }
        } else if (s_frame_index == 1u) {
            if (s_rx_byte == 0x40) { s_frame[1] = 0x40; s_frame_index = 2; }
            else { s_frame_index = 0; }
        } else {
            s_frame[s_frame_index++] = s_rx_byte;
            if (s_frame_index >= 32) {
                // 校验和与解析逻辑
                uint16_t checksum = 0xFFFF;
                for (int i = 0; i < 30; i++) checksum -= s_frame[i];
                if (checksum == (uint16_t)(s_frame[30] | (s_frame[31] << 8))) {
                    for (int i = 0; i < 14; i++) 
                        s_data.channel[i] = (uint16_t)(s_frame[2+i*2] | (s_frame[3+i*2] << 8));
                    s_data.last_update_ms = HAL_GetTick();
                    s_data.valid = true;
                    s_data.frame_count++;
                }
                s_frame_index = 0;
            }
        }
        HAL_UART_Receive_IT(&huart5, &s_rx_byte, 1);
    }
}

void ibus_error_callback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART5) {
        s_data.error_count++;
        HAL_UART_AbortReceive_IT(&huart5);
        HAL_UART_Receive_IT(&huart5, &s_rx_byte, 1);
    }
}

bool ibus_get_data(FsIa10bData* out) {
    __disable_irq();
    *out = s_data;
    __enable_irq();
    return out->valid;
}

bool ibus_is_online(uint32_t timeout_ms) {
    return (s_data.valid && (HAL_GetTick() - s_data.last_update_ms < timeout_ms));
}

uint16_t ibus_get_channel(uint8_t index) {
    if (index >= 14) return 0;
    return s_data.channel[index];
}