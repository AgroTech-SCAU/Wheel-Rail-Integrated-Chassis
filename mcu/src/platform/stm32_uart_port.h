#ifndef _stm32_uart_port_h_
#define _stm32_uart_port_h_

/**
 * @file stm32_uart_port.h
 * @brief STM32 UART 接收与日志输出适配接口
 */

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief UART 事件回调函数类型
 * @param context 注册时绑定的用户上下文
 */
typedef void (*Stm32UartEventCallback)(void* context);

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 启动 UART5 中断接收
 * @param data 接收缓冲区
 * @param len 接收长度
 * @return bool `true` 表示启动成功
 */
bool stm32_uart5_start_receive(uint8_t* data, uint16_t len);

/**
 * @brief 中止 UART5 中断接收
 * @return bool `true` 表示操作成功
 */
bool stm32_uart5_abort_receive(void);

/**
 * @brief 判断 UART5 接收状态是否空闲
 * @return bool `true` 表示接收状态空闲
 */
bool stm32_uart5_receive_is_ready(void);

/**
 * @brief 注册 UART5 接收完成与错误回调
 * @param rx_complete 接收完成回调
 * @param error 错误回调
 * @param context 用户上下文
 * @return bool `true` 表示注册成功
 */
bool stm32_uart5_register_callbacks(Stm32UartEventCallback rx_complete,
                                    Stm32UartEventCallback error,
                                    void* context);

/**
 * @brief 通过 USART1 同步写入日志数据
 * @param data 待发送数据
 * @param len 数据长度
 * @return bool `true` 表示发送成功
 */
bool stm32_usart1_write(const char* data, uint32_t len);

#endif
