#include "main.h"
#include "BlueSerial.h"
#include "usart.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* 接收完整数据包后，内容存到这里
 * 例如手机发送：[key,1,up]
 * 这里最终得到：key,1,up
 */
char BlueSerial_RxPacket[BLUESERIAL_RX_PACKET_SIZE];

/* 收到完整一包后置 1，主循环解析完后清 0 */
volatile uint8_t BlueSerial_RxFlag = 0;

/* HAL_UART_Receive_IT 需要一个长期有效的接收字节缓存，不能用局部变量 */
static uint8_t BlueSerial_RxByte = 0;


/**
  * @brief  蓝牙串口初始化
  * @note   必须在 MX_USART1_UART_Init() 之后调用
  */
void BlueSerial_Init(void)
{
    BlueSerial_RxFlag = 0;
    BlueSerial_RxPacket[0] = '\0';
    BlueSerial_RxByte = 0;

    /* 启动第一次 USART1 中断接收 */
    HAL_UART_Receive_IT(&huart1, &BlueSerial_RxByte, 1);
}


/**
  * @brief  USART1 发送一个字节
  */
void BlueSerial_SendByte(uint8_t Byte)
{
    HAL_UART_Transmit(&huart1, &Byte, 1, HAL_MAX_DELAY);
}


/**
  * @brief  USART1 发送数组
  */
void BlueSerial_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;

    for (i = 0; i < Length; i++)
    {
        BlueSerial_SendByte(Array[i]);
    }
}


/**
  * @brief  USART1 发送字符串
  */
void BlueSerial_SendString(char *String)
{
    uint16_t i;

    for (i = 0; String[i] != '\0'; i++)
    {
        BlueSerial_SendByte((uint8_t)String[i]);
    }
}


/**
  * @brief  幂函数，用于发送数字
  */
uint32_t BlueSerial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;

    while (Y--)
    {
        Result *= X;
    }

    return Result;
}


/**
  * @brief  发送指定长度的数字
  */
void BlueSerial_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;

    for (i = 0; i < Length; i++)
    {
        BlueSerial_SendByte(Number / BlueSerial_Pow(10, Length - i - 1) % 10 + '0');
    }
}


/**
  * @brief  格式化发送，等价于你原来的 Serial_Printf
  */
void BlueSerial_Printf(char *format, ...)
{
    char String[100];
    va_list arg;

    va_start(arg, format);
    vsnprintf(String, sizeof(String), format, arg);
    va_end(arg);

    BlueSerial_SendString(String);
}


/**
  * @brief  printf 重定向到 USART1
  * @note   如果你直接使用 printf("xxx")，也会从 USART1 发出去
  */
int fputc(int ch, FILE *f)
{
    (void)f;

    BlueSerial_SendByte((uint8_t)ch);

    return ch;
}


/**
  * @brief  GCC / CubeIDE 下的 printf 重定向
  * @note   如果你的工程 syscalls.c 里已经有 _write，
  *         需要把 syscalls.c 里的 _write 内容改成这里的发送逻辑，
  *         否则 printf 可能不会从 USART1 输出。
  */
#if defined(__GNUC__)
__attribute__((weak)) int _write(int file, char *ptr, int len)
{
    (void)file;

    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);

    return len;
}
#endif


// /**
//   * @brief  HAL 串口接收完成回调函数
//   * @note   每收到 1 个字节进入一次
//   *         协议保持和你标准库代码一致：
//   *         '[' 开始，']' 结束，中间内容存入 BlueSerial_RxPacket
//   */
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//     static uint8_t RxState = 0;
//     static uint8_t pRxPacket = 0;

//     if (huart->Instance == USART1)
//     {
//         uint8_t RxData = BlueSerial_RxByte;

//         if (RxState == 0)
//         {
//             if (RxData == '[' && BlueSerial_RxFlag == 0)
//             {
//                 RxState = 1;
//                 pRxPacket = 0;
//             }
//         }
//         else if (RxState == 1)
//         {
//             if (RxData == ']')
//             {
//                 RxState = 0;
//                 BlueSerial_RxPacket[pRxPacket] = '\0';
//                 BlueSerial_RxFlag = 1;
//             }
//             else
//             {
//                 if (pRxPacket < BLUESERIAL_RX_PACKET_SIZE - 1)
//                 {
//                     BlueSerial_RxPacket[pRxPacket] = (char)RxData;
//                     pRxPacket++;
//                 }
//                 else
//                 {
//                     /* 数据包过长，丢弃本包，防止数组越界 */
//                     RxState = 0;
//                     pRxPacket = 0;
//                     BlueSerial_RxPacket[0] = '\0';
//                 }
//             }
//         }

//         /* 关键：重新开启下一字节接收，否则只能收 1 个字节 */
//         HAL_UART_Receive_IT(&huart1, &BlueSerial_RxByte, 1);
//     }
// }


// /**
//   * @brief  串口错误回调
//   * @note   出现溢出、噪声、帧错误后，重新开启接收，提高稳定性
//   */
// void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
// {
//     if (huart->Instance == USART1)
//     {
//         HAL_UART_Receive_IT(&huart1, &BlueSerial_RxByte, 1);
//     }
// }