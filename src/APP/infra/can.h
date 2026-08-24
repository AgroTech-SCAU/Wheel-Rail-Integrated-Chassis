#ifndef _can_h_
#define _can_h_

#include <stdbool.h>
#include <stdint.h>
#include <string.h> // IWYU pragma: keep
#include "main.h"   // IWYU pragma: keep
#include "fdcan.h"  // IWYU pragma: keep

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @def CAN_BUS_TX_TIMEOUT_MS
 * @brief 默认 CAN 发送等待超时时间，单位 ms。
 *
 * 当发送 FIFO 已满时，发送函数会在该时间内等待 FIFO 释放。
 * 如果工程中需要调整默认等待时间，可以在编译选项或包含本头文件前重新定义。
 */
#ifndef CAN_BUS_TX_TIMEOUT_MS
#define CAN_BUS_TX_TIMEOUT_MS 10U
#endif

/**
 * @brief Classical CAN 单帧最大数据长度，单位 byte。
 *
 * 当前工程的 RS06 通信使用 Classical CAN 格式，因此固定为 8 byte。
 */
#define CAN_BUS_MAX_CLASSIC_DATA_LEN 8U

/**
 * @brief CAN 总线通用状态码。
 */
typedef enum {
    CAN_BUS_OK = 0,       /**< 操作成功 */
    CAN_BUS_ERROR,        /**< HAL 底层返回错误 */
    CAN_BUS_BUSY,         /**< 总线或外设忙，当前版本预留 */
    CAN_BUS_TIMEOUT,      /**< 等待发送 FIFO 超时 */
    CAN_BUS_INVALID_ARG,  /**< 输入参数无效 */
} can_bus_status_t;

/**
 * @brief CAN 标识符类型。
 */
typedef enum {
    CAN_BUS_ID_STD = 0,   /**< 11 bit 标准帧 ID */
    CAN_BUS_ID_EXT = 1,   /**< 29 bit 扩展帧 ID */
} can_bus_id_type_t;

/**
 * @brief CAN 总线帧数据结构。
 *
 * 该结构只描述 Classical CAN 数据帧，不描述 CAN FD 大负载帧。
 */
typedef struct {
    uint32_t id;                                  /**< CAN ID，标准帧最大 0x7FF，扩展帧最大 0x1FFFFFFF */
    can_bus_id_type_t id_type;                    /**< 标准帧或扩展帧 */
    uint8_t len;                                  /**< 数据长度，范围 0~8 */
    uint8_t data[CAN_BUS_MAX_CLASSIC_DATA_LEN];   /**< 数据区 */
} can_bus_frame_t;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

uint32_t can_bus_len_to_dlc(uint8_t len);
uint8_t can_bus_dlc_to_len(uint32_t dlc);

can_bus_status_t can_bus_init(FDCAN_HandleTypeDef* hfdcan);
can_bus_status_t can_bus_send(FDCAN_HandleTypeDef* hfdcan, const can_bus_frame_t* frame, uint32_t timeout_ms);
can_bus_status_t can_bus_send_std(FDCAN_HandleTypeDef* hfdcan, uint32_t std_id, const uint8_t* data, uint8_t len, uint32_t timeout_ms);
can_bus_status_t can_bus_send_ext(FDCAN_HandleTypeDef* hfdcan, uint32_t ext_id, const uint8_t* data, uint8_t len, uint32_t timeout_ms);
can_bus_status_t can_bus_receive_fifo0(FDCAN_HandleTypeDef* hfdcan, can_bus_frame_t* frame);

void can_filter_init(FDCAN_HandleTypeDef* fdcanHandle);
void can_send(FDCAN_HandleTypeDef* hfdcanx, uint32_t id, uint8_t* data, uint8_t len);

#endif
