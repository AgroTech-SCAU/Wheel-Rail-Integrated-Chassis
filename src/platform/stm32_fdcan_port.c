/**
 * @file stm32_fdcan_port.c
 * @brief STM32 FDCAN 总线适配实现
 */

#include "stm32_fdcan_port.h"

#include "fdcan.h"
#include "main.h"

#include <stddef.h>
#include <string.h>

#ifndef FDCAN_REJECT_REMOTE
#define FDCAN_REJECT_REMOTE FDCAN_FILTER_REMOTE
#endif

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 单条 FDCAN 接收回调注册槽位
 */
typedef struct {
    Stm32FdcanRxCallback callback;
    void* context;
} Stm32FdcanCallbackSlot;

static Stm32FdcanCallbackSlot s_rx_slots[STM32_FDCAN_BUS_COUNT];

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 将总线角色映射到 CubeMX FDCAN 句柄
 */
static FDCAN_HandleTypeDef* stm32_fdcan_handle(Stm32FdcanBus bus) {
    if(bus == STM32_FDCAN_BUS_DRIVE) {
        return &hfdcan1;
    }
    if(bus == STM32_FDCAN_BUS_STEER) {
        return &hfdcan2;
    }
    return NULL;
}

/**
 * @brief 将经典 CAN 字节长度转换为 FDCAN DLC
 */
static uint32_t stm32_fdcan_len_to_dlc(uint8_t len) {
    static const uint32_t dlc[] = {
        FDCAN_DLC_BYTES_0,
        FDCAN_DLC_BYTES_1,
        FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3,
        FDCAN_DLC_BYTES_4,
        FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6,
        FDCAN_DLC_BYTES_7,
        FDCAN_DLC_BYTES_8,
    };
    return len <= STM32_FDCAN_CLASSIC_DATA_LEN ? dlc[len] : FDCAN_DLC_BYTES_0;
}

/**
 * @brief 将 FDCAN DLC 转换为经典 CAN 字节长度
 */
static uint8_t stm32_fdcan_dlc_to_len(uint32_t dlc) {
    switch(dlc) {
        case FDCAN_DLC_BYTES_0:
            return 0u;
        case FDCAN_DLC_BYTES_1:
            return 1u;
        case FDCAN_DLC_BYTES_2:
            return 2u;
        case FDCAN_DLC_BYTES_3:
            return 3u;
        case FDCAN_DLC_BYTES_4:
            return 4u;
        case FDCAN_DLC_BYTES_5:
            return 5u;
        case FDCAN_DLC_BYTES_6:
            return 6u;
        case FDCAN_DLC_BYTES_7:
            return 7u;
        case FDCAN_DLC_BYTES_8:
            return 8u;
        default:
            return 0u;
    }
}

// ! ========================= 接 口 函 数 实 现 ========================= ! //

bool stm32_fdcan_port_init(Stm32FdcanBus bus) {
    FDCAN_HandleTypeDef* handle = stm32_fdcan_handle(bus);
    if(handle == NULL) {
        return false;
    }
    if(handle->State == HAL_FDCAN_STATE_BUSY) {
        return true;
    }

    if(bus == STM32_FDCAN_BUS_DRIVE) {
        HAL_GPIO_WritePin(CAN1_EN_GPIO_Port, CAN1_EN_Pin, GPIO_PIN_SET);
    }
    else {
        HAL_GPIO_WritePin(CAN2_EN_GPIO_Port, CAN2_EN_Pin, GPIO_PIN_SET);
    }
    if(HAL_FDCAN_ConfigGlobalFilter(
           handle, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
           FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) {
        return false;
    }
    if(HAL_FDCAN_ActivateNotification(handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                      0u) != HAL_OK) {
        return false;
    }
    return HAL_FDCAN_Start(handle) == HAL_OK;
}

static bool stm32_fdcan_send(Stm32FdcanBus bus, uint32_t id,
                             Stm32FdcanIdType id_type, const uint8_t* data,
                             uint8_t len, uint32_t timeout_ms) {
    FDCAN_HandleTypeDef* handle = stm32_fdcan_handle(bus);
    FDCAN_TxHeaderTypeDef header;
    uint32_t start_ms;
    if(handle == NULL || len > STM32_FDCAN_CLASSIC_DATA_LEN ||
       (len > 0u && data == NULL) ||
       (id_type == STM32_FDCAN_ID_STANDARD && id > 0x7FFu) ||
       (id_type == STM32_FDCAN_ID_EXTENDED && id > 0x1FFFFFFFu)) {
        return false;
    }
    if(!stm32_fdcan_port_init(bus)) {
        return false;
    }
    start_ms = HAL_GetTick();
    while(HAL_FDCAN_GetTxFifoFreeLevel(handle) == 0u) {
        if(timeout_ms == 0u ||
           (uint32_t)(HAL_GetTick() - start_ms) >= timeout_ms) {
            return false;
        }
    }
    memset(&header, 0, sizeof(header));
    header.Identifier = id;
    header.IdType = id_type == STM32_FDCAN_ID_EXTENDED ? FDCAN_EXTENDED_ID
                                                       : FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = stm32_fdcan_len_to_dlc(len);
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    return HAL_FDCAN_AddMessageToTxFifoQ(handle, &header, (uint8_t*)data) ==
           HAL_OK;
}

bool stm32_fdcan_port_send_standard(Stm32FdcanBus bus, uint32_t id,
                                    const uint8_t* data, uint8_t len,
                                    uint32_t timeout_ms) {
    return stm32_fdcan_send(bus, id, STM32_FDCAN_ID_STANDARD, data, len,
                            timeout_ms);
}

bool stm32_fdcan_port_send_extended(Stm32FdcanBus bus, uint32_t id,
                                    const uint8_t* data, uint8_t len,
                                    uint32_t timeout_ms) {
    return stm32_fdcan_send(bus, id, STM32_FDCAN_ID_EXTENDED, data, len,
                            timeout_ms);
}

bool stm32_fdcan_port_register_rx_callback(Stm32FdcanBus bus,
                                           Stm32FdcanRxCallback callback,
                                           void* context) {
    if(bus >= STM32_FDCAN_BUS_COUNT || callback == NULL) {
        return false;
    }
    if(s_rx_slots[bus].callback != NULL &&
       (s_rx_slots[bus].callback != callback ||
        s_rx_slots[bus].context != context)) {
        return false;
    }
    s_rx_slots[bus].callback = callback;
    s_rx_slots[bus].context = context;
    return true;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* handle,
                               uint32_t interrupts) {
    Stm32FdcanBus bus;
    if(handle == NULL || (interrupts & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0u) {
        return;
    }
    if(handle->Instance == FDCAN1) {
        bus = STM32_FDCAN_BUS_DRIVE;
    }
    else if(handle->Instance == FDCAN2) {
        bus = STM32_FDCAN_BUS_STEER;
    }
    else {
        return;
    }

    while(HAL_FDCAN_GetRxFifoFillLevel(handle, FDCAN_RX_FIFO0) > 0u) {
        FDCAN_RxHeaderTypeDef header;
        Stm32FdcanFrame frame;
        uint8_t data[STM32_FDCAN_CLASSIC_DATA_LEN] = { 0u };
        if(HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0, &header, data) !=
           HAL_OK) {
            return;
        }
        memset(&frame, 0, sizeof(frame));
        frame.id = header.Identifier;
        frame.id_type = header.IdType == FDCAN_EXTENDED_ID
                            ? STM32_FDCAN_ID_EXTENDED
                            : STM32_FDCAN_ID_STANDARD;
        frame.len = stm32_fdcan_dlc_to_len(header.DataLength);
        memcpy(frame.data, data, frame.len);
        if(s_rx_slots[bus].callback != NULL) {
            s_rx_slots[bus].callback(&frame, s_rx_slots[bus].context);
        }
    }
}
