#ifndef _stm32_fdcan_port_h_
#define _stm32_fdcan_port_h_

/**
 * @file stm32_fdcan_port.h
 * @brief STM32 FDCAN 总线适配接口
 */

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 经典 CAN 数据区最大长度
 */
#define STM32_FDCAN_CLASSIC_DATA_LEN 8u

/**
 * @brief 项目使用的 FDCAN 总线角色
 */
typedef enum {
    STM32_FDCAN_BUS_DRIVE = 0,
    STM32_FDCAN_BUS_STEER,
    STM32_FDCAN_BUS_COUNT,
} Stm32FdcanBus;

/**
 * @brief CAN 标识符类型
 */
typedef enum {
    STM32_FDCAN_ID_STANDARD = 0,
    STM32_FDCAN_ID_EXTENDED,
} Stm32FdcanIdType;

/**
 * @brief 经典 CAN 接收帧
 */
typedef struct {
    uint32_t id;
    Stm32FdcanIdType id_type;
    uint8_t len;
    uint8_t data[STM32_FDCAN_CLASSIC_DATA_LEN];
} Stm32FdcanFrame;

/**
 * @brief FDCAN 接收回调函数类型
 * @param frame 接收帧只读指针
 * @param context 注册时绑定的用户上下文
 */
typedef void (*Stm32FdcanRxCallback)(const Stm32FdcanFrame* frame, void* context);

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化并启动指定 FDCAN 总线
 * @param bus 总线角色
 * @return bool `true` 表示初始化成功
 */
bool stm32_fdcan_port_init(Stm32FdcanBus bus);

/**
 * @brief 发送标准标识符经典 CAN 帧
 * @param bus 总线角色
 * @param id 标准帧 ID
 * @param data 数据缓冲区
 * @param len 数据长度
 * @param timeout_ms 等待发送 FIFO 的最长时间
 * @return bool `true` 表示帧已加入发送 FIFO
 */
bool stm32_fdcan_port_send_standard(Stm32FdcanBus bus,
                                    uint32_t id,
                                    const uint8_t* data,
                                    uint8_t len,
                                    uint32_t timeout_ms);
/**
 * @brief 发送扩展标识符经典 CAN 帧
 * @param bus 总线角色
 * @param id 扩展帧 ID
 * @param data 数据缓冲区
 * @param len 数据长度
 * @param timeout_ms 等待发送 FIFO 的最长时间
 * @return bool `true` 表示帧已加入发送 FIFO
 */
bool stm32_fdcan_port_send_extended(Stm32FdcanBus bus,
                                    uint32_t id,
                                    const uint8_t* data,
                                    uint8_t len,
                                    uint32_t timeout_ms);
/**
 * @brief 注册指定总线的接收回调
 * @param bus 总线角色
 * @param callback 接收回调
 * @param context 用户上下文
 * @return bool `true` 表示注册成功
 */
bool stm32_fdcan_port_register_rx_callback(Stm32FdcanBus bus,
                                           Stm32FdcanRxCallback callback,
                                           void* context);

#endif
