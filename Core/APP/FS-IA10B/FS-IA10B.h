#ifndef _FS_IA10B_H_
#define _FS_IA10B_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h" // 针对 STM32H7 系列

#define FS_IA10B_IBUS_FRAME_LEN 32u
#define FS_IA10B_CHANNEL_COUNT 14u

typedef struct {
    uint16_t channel[FS_IA10B_CHANNEL_COUNT];
    uint32_t frame_count;
    uint32_t error_count;
    uint32_t last_update_ms;
    bool valid;
} FsIa10bData;

void ibus_init(void);
void ibus_maintain(void);
bool ibus_get_data(FsIa10bData* out);
bool ibus_is_online(uint32_t timeout_ms);
uint16_t ibus_get_channel(uint8_t index);

// 供 STM32 HAL 回调调用的接口
void ibus_rx_complete_callback(UART_HandleTypeDef *huart);
void ibus_error_callback(UART_HandleTypeDef *huart);

#endif