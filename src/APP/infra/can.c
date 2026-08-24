#include "can.h"

#ifndef FDCAN_REJECT_REMOTE
#define FDCAN_REJECT_REMOTE FDCAN_FILTER_REMOTE
#endif

/**
 * @brief 判断 FDCAN 外设是否已经处于启动状态
 * @param hfdcan FDCAN 句柄
 * @retval true 外设已启动
 * @retval false 外设未启动或句柄为空
 */
static bool can_bus_is_init(FDCAN_HandleTypeDef* hfdcan) {
    return hfdcan != NULL && hfdcan->State == HAL_FDCAN_STATE_BUSY;
}

/**
 * @brief 将字节长度转换为 STM32 HAL FDCAN DLC 枚举值
 * @param len 数据长度，范围 0~8
 * @return 对应的 FDCAN_DLC_BYTES_x；非法长度返回 0
 */
uint32_t can_bus_len_to_dlc(uint8_t len) {
    switch(len) {
        case 0: return FDCAN_DLC_BYTES_0;
        case 1: return FDCAN_DLC_BYTES_1;
        case 2: return FDCAN_DLC_BYTES_2;
        case 3: return FDCAN_DLC_BYTES_3;
        case 4: return FDCAN_DLC_BYTES_4;
        case 5: return FDCAN_DLC_BYTES_5;
        case 6: return FDCAN_DLC_BYTES_6;
        case 7: return FDCAN_DLC_BYTES_7;
        case 8: return FDCAN_DLC_BYTES_8;
        default: return 0U;
    }
}

/**
 * @brief 将 STM32 HAL FDCAN DLC 枚举值转换为字节长度
 * @param dlc FDCAN_RxHeaderTypeDef 或 FDCAN_TxHeaderTypeDef 中的 DataLength
 * @return 对应字节长度；不支持的 DLC 返回 0
 */
uint8_t can_bus_dlc_to_len(uint32_t dlc) {
    switch(dlc) {
        case FDCAN_DLC_BYTES_0: return 0;
        case FDCAN_DLC_BYTES_1: return 1;
        case FDCAN_DLC_BYTES_2: return 2;
        case FDCAN_DLC_BYTES_3: return 3;
        case FDCAN_DLC_BYTES_4: return 4;
        case FDCAN_DLC_BYTES_5: return 5;
        case FDCAN_DLC_BYTES_6: return 6;
        case FDCAN_DLC_BYTES_7: return 7;
        case FDCAN_DLC_BYTES_8: return 8;
        default: return 0;
    }
}

/**
 * @brief 启动 FDCAN 外设并配置基础接收过滤
 *
 * 当前配置会把未匹配的标准帧与扩展帧都接收到 FIFO0，便于调试 RS06 等扩展帧设备
 * 远程帧会被拒收或过滤
 *
 * @param hfdcan FDCAN 句柄
 * @return CAN_BUS_OK 表示启动成功；其他值表示失败原因
 */
can_bus_status_t can_bus_init(FDCAN_HandleTypeDef* hfdcan) {
    if(hfdcan == NULL) {
        return CAN_BUS_INVALID_ARG;
    }

    if(can_bus_is_init(hfdcan)) {
        return CAN_BUS_OK;
    }

    HAL_GPIO_WritePin(CAN1_EN_GPIO_Port, CAN1_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CAN2_EN_GPIO_Port, CAN2_EN_Pin, GPIO_PIN_SET);

    if(HAL_FDCAN_ConfigGlobalFilter(hfdcan,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE) != HAL_OK) {
        return CAN_BUS_ERROR;
    }

    if(HAL_FDCAN_ActivateNotification(hfdcan,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
        0U) != HAL_OK) {
        return CAN_BUS_ERROR;
    }

    if(HAL_FDCAN_Start(hfdcan) != HAL_OK) {
        return CAN_BUS_ERROR;
    }

    return CAN_BUS_OK;
}

/**
 * @brief 发送一帧 Classical CAN 数据帧
 *
 * 函数会自动检查 ID 范围、数据长度，并在 TX FIFO 满时等待指定超时时间
 * 如果 FDCAN 尚未启动，会先尝试调用 can_bus_init()
 *
 * @param hfdcan FDCAN 句柄
 * @param frame 待发送帧
 * @param timeout_ms 等待 TX FIFO 空位的超时时间，0 表示不等待
 * @return CAN_BUS_OK 表示发送成功；其他值表示失败原因
 */
can_bus_status_t can_bus_send(FDCAN_HandleTypeDef* hfdcan, const can_bus_frame_t* frame, uint32_t timeout_ms) {
    if(hfdcan == NULL || frame == NULL) {
        return CAN_BUS_INVALID_ARG;
    }

    if(frame->len > CAN_BUS_MAX_CLASSIC_DATA_LEN) {
        return CAN_BUS_INVALID_ARG;
    }

    if(frame->id_type == CAN_BUS_ID_STD && frame->id > 0x7FFU) {
        return CAN_BUS_INVALID_ARG;
    }

    if(frame->id_type == CAN_BUS_ID_EXT && frame->id > 0x1FFFFFFFU) {
        return CAN_BUS_INVALID_ARG;
    }

    uint32_t dlc = can_bus_len_to_dlc(frame->len);
    if(dlc == 0U && frame->len != 0U) {
        return CAN_BUS_INVALID_ARG;
    }

    if(!can_bus_is_init(hfdcan)) {
        can_bus_status_t st = can_bus_init(hfdcan);
        if(st != CAN_BUS_OK) {
            return st;
        }
    }

    const uint32_t start = HAL_GetTick();
    while(HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0U) {
        if(timeout_ms == 0U || (HAL_GetTick() - start) >= timeout_ms) {
            return CAN_BUS_TIMEOUT;
        }
    }

    FDCAN_TxHeaderTypeDef tx_header;
    memset(&tx_header, 0, sizeof(tx_header));
    tx_header.Identifier = frame->id;
    tx_header.IdType = (frame->id_type == CAN_BUS_ID_EXT) ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = dlc;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0U;

    if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, (uint8_t*)frame->data) != HAL_OK) {
        return CAN_BUS_ERROR;
    }

    return CAN_BUS_OK;
}

/**
 * @brief 发送一帧标准 ID Classical CAN 数据帧
 * @param hfdcan FDCAN 句柄
 * @param std_id 11 bit 标准帧 ID
 * @param data 数据区指针；当 len 为 0 时可为 NULL
 * @param len 数据长度，范围 0~8
 * @param timeout_ms 等待 TX FIFO 空位的超时时间，0 表示不等待
 * @return CAN_BUS_OK 表示发送成功；其他值表示失败原因
 */
can_bus_status_t can_bus_send_std(FDCAN_HandleTypeDef* hfdcan, uint32_t std_id, const uint8_t* data, uint8_t len, uint32_t timeout_ms) {
    can_bus_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.id = std_id;
    frame.id_type = CAN_BUS_ID_STD;
    frame.len = len;

    if(data != NULL && len <= CAN_BUS_MAX_CLASSIC_DATA_LEN) {
        memcpy(frame.data, data, len);
    }

    return can_bus_send(hfdcan, &frame, timeout_ms);
}

/**
 * @brief 发送一帧扩展 ID Classical CAN 数据帧
 * @param hfdcan FDCAN 句柄
 * @param ext_id 29 bit 扩展帧 ID
 * @param data 数据区指针；当 len 为 0 时可为 NULL
 * @param len 数据长度，范围 0~8
 * @param timeout_ms 等待 TX FIFO 空位的超时时间，0 表示不等待
 * @return CAN_BUS_OK 表示发送成功；其他值表示失败原因
 */
can_bus_status_t can_bus_send_ext(FDCAN_HandleTypeDef* hfdcan, uint32_t ext_id, const uint8_t* data, uint8_t len, uint32_t timeout_ms) {
    can_bus_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.id = ext_id;
    frame.id_type = CAN_BUS_ID_EXT;
    frame.len = len;

    if(data != NULL && len <= CAN_BUS_MAX_CLASSIC_DATA_LEN) {
        memcpy(frame.data, data, len);
    }

    return can_bus_send(hfdcan, &frame, timeout_ms);
}

/**
 * @brief 从 FDCAN RX FIFO0 读取一帧数据
 *
 * 通常在 HAL_FDCAN_RxFifo0Callback() 或主循环轮询中调用
 *
 * @param hfdcan FDCAN 句柄
 * @param frame 输出帧缓存
 * @return CAN_BUS_OK 表示读取成功；其他值表示失败原因
 */
can_bus_status_t can_bus_receive_fifo0(FDCAN_HandleTypeDef* hfdcan, can_bus_frame_t* frame) {
    if(hfdcan == NULL || frame == NULL) {
        return CAN_BUS_INVALID_ARG;
    }

    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t data[CAN_BUS_MAX_CLASSIC_DATA_LEN] = { 0 };

    if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, data) != HAL_OK) {
        return CAN_BUS_ERROR;
    }

    memset(frame, 0, sizeof(*frame));
    frame->id = rx_header.Identifier;
    frame->id_type = (rx_header.IdType == FDCAN_EXTENDED_ID) ? CAN_BUS_ID_EXT : CAN_BUS_ID_STD;
    frame->len = can_bus_dlc_to_len(rx_header.DataLength);

    if(frame->len > CAN_BUS_MAX_CLASSIC_DATA_LEN) {
        return CAN_BUS_ERROR;
    }

    memcpy(frame->data, data, frame->len);

    return CAN_BUS_OK;
}

/**
 * @brief 兼容旧工程的 CAN 初始化入口
 * @param fdcanHandle FDCAN 句柄
 */
void can_filter_init(FDCAN_HandleTypeDef* fdcanHandle) {
    if(can_bus_init(fdcanHandle) != CAN_BUS_OK) {
        Error_Handler();
    }
}

/**
 * @brief 兼容旧工程的 CAN 发送入口
 * @param hfdcanx FDCAN 句柄
 * @param id CAN ID，id > 0x7FF 时按扩展帧发送
 * @param data 数据区指针
 * @param len 数据长度，范围 0~8
 */
void can_send(FDCAN_HandleTypeDef* hfdcanx, uint32_t id, uint8_t* data, uint8_t len) {
    can_bus_id_type_t id_type = (id > 0x7FFU) ? CAN_BUS_ID_EXT : CAN_BUS_ID_STD;
    can_bus_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.id = id;
    frame.id_type = id_type;
    frame.len = len;

    if(data != NULL && len <= CAN_BUS_MAX_CLASSIC_DATA_LEN) {
        memcpy(frame.data, data, len);
    }

    (void)can_bus_send(hfdcanx, &frame, CAN_BUS_TX_TIMEOUT_MS);
}
