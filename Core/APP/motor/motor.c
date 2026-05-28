#include "motor.h"
#include "fdcan.h"
#include <string.h>
#include "can.h"
#include "main.h"
#include "tim.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern TIM_HandleTypeDef htim6;

extern reporter Motor_Reporter_Data;

uint8_t RxData[8];

static uint8_t Motor_TxData_0x32[8] = {0};
static uint8_t Motor_TxData_0x33[8] = {0};

reporter Motor_Reporter_Cache[4];

uint8_t query_id = 1;

typedef struct {
    int16_t target_rpm;
    float current_rpm;
    float accel_step;
} Motor_Smooth_Ctrl_t;

static Motor_Smooth_Ctrl_t MotorStates[4];

/* 初始化 */
void Motor_Driver_Init(void)
{
    HAL_GPIO_WritePin(CAN1_EN_GPIO_Port, CAN1_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CAN2_EN_GPIO_Port, CAN2_EN_Pin, GPIO_PIN_SET);

    if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                    FDCAN_ACCEPT_IN_RX_FIFO0,
                                    FDCAN_ACCEPT_IN_RX_FIFO0,
                                    FDCAN_REJECT_REMOTE,
                                    FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
                                    FDCAN_ACCEPT_IN_RX_FIFO0,
                                    FDCAN_ACCEPT_IN_RX_FIFO0,
                                    FDCAN_REJECT_REMOTE,
                                    FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    for(int i = 0; i < 4; i++)
    {
        MotorStates[i].target_rpm = 0;
        MotorStates[i].current_rpm = 0.0f;
        MotorStates[i].accel_step = 5.0f;
    }

    HAL_TIM_Base_Start_IT(&htim6);
}

/* 底层发送 */
static void Motor_Drive(int16_t InputRPM, uint8_t ID)
{
    // 【修改点 1】：在底层统一处理 ID 3 和 4 的反向逻辑，确保所有控制接口表现一致
    if (ID == 3 || ID == 4)
    {
        InputRPM = -InputRPM;
    }

    int32_t target_val = (int32_t)InputRPM * 100; // 与手册协议中 2000 对应 07 D0 完全一致，这里不需要修改

    if(target_val > 32767)
        target_val = 32767;

    if(target_val < -32767)
        target_val = -32767;

    int16_t ScaledSpeed = (int16_t)target_val;

    if(ID >= 1 && ID <= 4)
    {
        uint8_t idx = (ID - 1) * 2;
        Motor_TxData_0x32[idx]     = (uint8_t)(ScaledSpeed >> 8);
        Motor_TxData_0x32[idx + 1] = (uint8_t)(ScaledSpeed & 0xFF);
    }
}

/* 普通控制 */
void Motor_Speed_Control(int16_t InputRPM, uint8_t ID)
{
    if(InputRPM > SPEED_RPM_MAX)
        InputRPM = SPEED_RPM_MAX;

    if(InputRPM < SPEED_RPM_MIN)
        InputRPM = SPEED_RPM_MIN;

    Motor_Drive(InputRPM, ID);
    can_send(&hfdcan1, 0x032, Motor_TxData_0x32, 8);
}

/* 平滑控制 - 接口和内部逻辑完全未变 */
void Motor_Speed_Control_Smooth(int16_t InputRPM, uint8_t ID)
{
    if(ID < 1 || ID > 4)
        return;

    if(InputRPM > SPEED_RPM_MAX)
        InputRPM = SPEED_RPM_MAX;

    if(InputRPM < SPEED_RPM_MIN)
        InputRPM = SPEED_RPM_MIN;

    MotorStates[ID - 1].target_rpm = InputRPM;
}

/* 四电机 */
void Motor_Control_All(int16_t target)
{
    for(uint8_t i = 1; i <= 4; i++)
    {
        // 【修改点 2】：底层已处理反向，这里直接保持最干净、统一的调用
        // 避免在此处再次取负数导致“负负得正”的二次反转 Bug
        Motor_Speed_Control_Smooth(target, i);
    }
}

/* 急停 */
void Motor_Stop_Immediately(uint8_t ID)
{
    if(ID < 1 || ID > 4)
        return;

    MotorStates[ID - 1].target_rpm = 0;
    MotorStates[ID - 1].current_rpm = 0;

    uint8_t idx = (ID - 1) * 2;

    Motor_TxData_0x32[idx] = 0;
    Motor_TxData_0x32[idx + 1] = 0;

    can_send(&hfdcan1, 0x032, Motor_TxData_0x32, 8);
}

/* TIM中断 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM6)
    {
        for(int i = 0; i < 4; i++)
        {
            float diff = MotorStates[i].target_rpm - MotorStates[i].current_rpm;

            if(diff > MotorStates[i].accel_step)
            {
                MotorStates[i].current_rpm += MotorStates[i].accel_step;
            }
            else if(diff < -MotorStates[i].accel_step)
            {
                MotorStates[i].current_rpm -= MotorStates[i].accel_step;
            }
            else
            {
                MotorStates[i].current_rpm = MotorStates[i].target_rpm;
            }

            Motor_Drive((int16_t)MotorStates[i].current_rpm, i + 1);
        }

        if(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0)
        {
            can_send(&hfdcan1, 0x032, Motor_TxData_0x32, 8);
        }
    }
}

/* 查询 */
void Ck_Check(uint8_t ID,
              uint8_t Check1,
              uint8_t Check2,
              uint8_t Check3,
              reporter* unused)
{
    uint8_t tx_buf[8];

    tx_buf[0] = ID;
    tx_buf[1] = Check1;
    tx_buf[2] = Check2;
    tx_buf[3] = Check3;
    tx_buf[4] = 0xAA;
    tx_buf[5] = 0;
    tx_buf[6] = 0;
    tx_buf[7] = 0;

    can_send(&hfdcan1, 0x107, tx_buf, 8);
}

/* 数据解析 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        FDCAN_RxHeaderTypeDef RxHeader;

        if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            uint8_t actual_id = (query_id == 1) ? 4 : (query_id - 1);

            if(actual_id >= 1 && actual_id <= 4)
            {
                Motor_Reporter_Cache[actual_id - 1].FBSpeed =
                (int16_t)((RxData[0] << 8) | RxData[1]);

                Motor_Reporter_Cache[actual_id - 1].Position =
                (uint16_t)((RxData[2] << 8) | RxData[3]);

                Motor_Reporter_Cache[actual_id - 1].ErrCode =
                RxData[4];
            }
        }
    }
}

/* 轮询读取 */
void App_Monitor_Read(void)
{
    Ck_Check(query_id, 1, 4, 5, NULL);

    query_id++;
    if(query_id > 4)
        query_id = 1;
}