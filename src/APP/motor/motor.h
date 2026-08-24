#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

/*
 * 本末驱动电机的速度给定范围，单位为 RPM。
 * CAN 协议字段的分辨率是 0.01 RPM，编码工作统一在 motor.c 内完成。
 */
#define SPEED_RPM_MAX              210
#define SPEED_RPM_MIN             (-210)
#define MOTOR_SMOOTH_STEP_RPM       5
#define MOTOR_SMOOTH_PERIOD_MS     10U

/*
 * 驱动电机节点 ID。这里统一使用十进制 1~4；它们在数值上分别等于
 * 0x01~0x04，但不要与下面的 CAN 报文标识符混为一谈。
 */
#define MOTOR_ID_1                  1U
#define MOTOR_ID_2                  2U
#define MOTOR_ID_3                  3U
#define MOTOR_ID_4                  4U
#define MOTOR_ID_MIN                MOTOR_ID_1
#define MOTOR_ID_MAX                MOTOR_ID_4
#define MOTOR_DRIVE_COUNT           4U

/* CAN 协议报文 ID，属于通信协议，继续使用十六进制表达。 */
#define MOTOR_SPEED_COMMAND_CAN_ID  0x032U
#define MOTOR_QUERY_CAN_ID          0x107U
#define MOTOR_CAN_DATA_LENGTH       8U

extern TIM_HandleTypeDef htim6;

typedef struct {
    int16_t FBSpeed;
    int16_t ECurru;
    int16_t Position;
    uint8_t ErrCode;
    uint8_t FBMode;
} reporter;

/*
 * 必须在 MX_FDCAN1_Init() 和 MX_TIM6_Init() 之后调用。
 * 函数可重复调用，内部只会初始化并启动一次 TIM6。
 */
void Motor_Driver_Init(void);

/* 基础功能 */
uint8_t Motor_Set_FeedBack(unsigned char FeedBack, uint8_t ID);
void Motor_Calibration(void);
void Ck_Check(uint8_t ID, uint8_t Check1, uint8_t Check2, uint8_t Check3, reporter* reporter);
void Obtain_Motor_Report(reporter* report);
uint8_t ID_Set(uint8_t ID);
uint8_t Motor_SetMode(unsigned char Mode);

/* 速度控制 */
void Motor_Speed_Control(int16_t InputRPM, uint8_t ID);
void Motor_Speed_Control_Smooth(int16_t InputRPM, uint8_t ID);

void Motor_Control_All(int16_t target);

/* 急停 */
void Motor_Stop_Immediately(uint8_t ID);

/* 回调 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

/* 数据读取 */
void App_Monitor_Read(void);

#endif
