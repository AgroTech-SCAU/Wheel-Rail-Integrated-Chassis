#include "main.h"
#include "BlueSerial.h"
#include "usart.h"
#include <stdio.h>
#include <stdarg.h>

// uint8_t Serial_RxData;
// uint8_t Serial_RxFlag;
char BlueSerial_RxPacket[100];
uint8_t BlueSerial_RxFlag;

void BlueSerial_SendByte(uint8_t Byte){
    HAL_UART_Transmit(&huart1, &Byte, 1, HAL_MAX_DELAY);
    while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == RESET);
}

void BlueSerial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		BlueSerial_SendByte(Array[i]);
	}
}

void BlueSerial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		BlueSerial_SendByte(String[i]);
	}
}

uint32_t BlueSerial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

void BlueSerial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		BlueSerial_SendByte(Number / BlueSerial_Pow(10, Length - i - 1) % 10 + '0');
	}
}

void BlueSerial_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	BlueSerial_SendString(String);
}

// uint8_t Serial_GetRxFlag(void)
// {
// 	if (Serial_RxFlag == 1)
// 	{
// 		Serial_RxFlag = 0;
// 		return 1;
// 	}
// 	return 0;
// }

// uint8_t Serial_GetRxData(void)
// {
// 	return Serial_RxData;
// }

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//     static uint8_t RxState = 0;
// 	static uint8_t pRxPacket = 0;
//     if (huart->Instance == USART3) {
//         uint8_t RxData;
//         HAL_UART_Receive_IT(&huart3, &RxData, 1);
//         if (RxState == 0)
// 		{
// 			if (RxData == '[' && BlueSerial_RxFlag == 0)
// 			{
// 				RxState = 1;
// 				pRxPacket = 0;
// 			}
// 		}
// 		else if (RxState == 1)
// 		{
// 			if (RxData == ']')
// 			{
// 				RxState = 0;
// 				BlueSerial_RxPacket[pRxPacket] = '\0';
// 				BlueSerial_RxFlag = 1;
// 			}
// 			else
// 			{
// 				BlueSerial_RxPacket[pRxPacket] = RxData;
// 				pRxPacket ++;
// 			}
// 		}
//     }
// }