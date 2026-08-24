#include "motor.h"
#include "fdcan.h"
#include <string.h>
#include "can.h"
#include "main.h"
#include "tim.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef htim6;

extern reporter Motor_Reporter_Data;

uint8_t RxData[8];

static uint8_t Motor_TxData_0x32[MOTOR_CAN_DATA_LENGTH] = {0};

reporter Motor_Reporter_Cache[MOTOR_DRIVE_COUNT];

uint8_t query_id = MOTOR_ID_1;
static volatile uint8_t Motor_Last_Query_ID = MOTOR_ID_1;

typedef struct {
    volatile int16_t target_rpm;
    volatile int16_t current_rpm;
    int16_t accel_step_rpm;
} Motor_Smooth_Ctrl_t;

static Motor_Smooth_Ctrl_t MotorStates[MOTOR_DRIVE_COUNT];
static uint8_t Motor_Driver_Initialized = 0U;
static volatile uint32_t Motor_Smooth_Tick_Count = 0U;
static volatile uint32_t Motor_Smooth_Tx_Error_Count = 0U;

static uint8_t Motor_ID_Is_Valid(uint8_t motor_id)
{
    return (motor_id >= MOTOR_ID_MIN && motor_id <= MOTOR_ID_MAX) ? 1U : 0U;
}

static int16_t Motor_Limit_RPM(int16_t InputRPM)
{
    if(InputRPM > SPEED_RPM_MAX)
    {
        return SPEED_RPM_MAX;
    }

    if(InputRPM < SPEED_RPM_MIN)
    {
        return SPEED_RPM_MIN;
    }

    return InputRPM;
}

static uint32_t Motor_Enter_Critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void Motor_Exit_Critical(uint32_t primask)
{
    if(primask == 0U)
    {
        __enable_irq();
    }
}

/* 初始化 */
void Motor_Driver_Init(void)
{
    if(Motor_Driver_Initialized != 0U)
    {
        return;
    }

    HAL_GPIO_WritePin(CAN1_EN_GPIO_Port, CAN1_EN_Pin, GPIO_PIN_SET);

    if(can_bus_init(&hfdcan1) != CAN_BUS_OK)
    {
        Error_Handler();
    }

    for(uint8_t i = 0U; i < MOTOR_DRIVE_COUNT; i++)
    {
        MotorStates[i].target_rpm = 0;
        MotorStates[i].current_rpm = 0;
        MotorStates[i].accel_step_rpm = MOTOR_SMOOTH_STEP_RPM;
    }

    memset(Motor_TxData_0x32, 0, sizeof(Motor_TxData_0x32));
    Motor_Smooth_Tick_Count = 0U;
    Motor_Smooth_Tx_Error_Count = 0U;

    /* 先置初始化标志，防止定时器启动后第一拍与初始化过程竞争。 */
    Motor_Driver_Initialized = 1U;
    __HAL_TIM_SET_COUNTER(&htim6, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);

    if(HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
    {
        Motor_Driver_Initialized = 0U;
        Error_Handler();
    }
}

/* 底层发送 */
static void Motor_Drive(int16_t InputRPM, uint8_t ID)
{
    /* ID 3、4 对应反向安装的驱动轮，方向统一在最底层处理。 */
    if(ID == MOTOR_ID_3 || ID == MOTOR_ID_4)
    {
        InputRPM = -InputRPM;
    }

    /* CAN 原始值单位为 0.01 RPM，例如 20 RPM -> 2000 -> 0x07D0。 */
    int32_t target_val = (int32_t)InputRPM * 100;

    if(target_val > 32767)
        target_val = 32767;

    if(target_val < -32767)
        target_val = -32767;

    int16_t ScaledSpeed = (int16_t)target_val;

    if(Motor_ID_Is_Valid(ID) != 0U)
    {
        uint8_t idx = (uint8_t)((ID - MOTOR_ID_MIN) * 2U);
        Motor_TxData_0x32[idx]     = (uint8_t)(ScaledSpeed >> 8);
        Motor_TxData_0x32[idx + 1] = (uint8_t)(ScaledSpeed & 0xFF);
    }
}

/* 普通控制 */
void Motor_Speed_Control(int16_t InputRPM, uint8_t ID)
{
    uint32_t primask;
    can_bus_status_t status;

    if(Motor_ID_Is_Valid(ID) == 0U)
    {
        return;
    }

    if(Motor_Driver_Initialized == 0U)
    {
        Motor_Driver_Init();
    }

    InputRPM = Motor_Limit_RPM(InputRPM);

    /*
     * 普通控制是立即跳变，但必须同步斜坡状态；否则下一次 TIM6
     * 中断会把旧的 current_rpm 再次发出，形成非零/旧值交替的顿挫。
     */
    primask = Motor_Enter_Critical();
    MotorStates[ID - MOTOR_ID_MIN].target_rpm = InputRPM;
    MotorStates[ID - MOTOR_ID_MIN].current_rpm = InputRPM;

    Motor_Drive(InputRPM, ID);
    status = can_bus_send_std(&hfdcan1,
                              MOTOR_SPEED_COMMAND_CAN_ID,
                              Motor_TxData_0x32,
                              sizeof(Motor_TxData_0x32),
                              0U);
    Motor_Exit_Critical(primask);

    if(status != CAN_BUS_OK)
    {
        Motor_Smooth_Tx_Error_Count++;
    }
}

/* 平滑控制：这里只更新目标，TIM6 每 10 ms 计算一次斜坡并发送。 */
void Motor_Speed_Control_Smooth(int16_t InputRPM, uint8_t ID)
{
    if(Motor_ID_Is_Valid(ID) == 0U)
    {
        return;
    }

    if(Motor_Driver_Initialized == 0U)
    {
        Motor_Driver_Init();
    }

    InputRPM = Motor_Limit_RPM(InputRPM);

    MotorStates[ID - MOTOR_ID_MIN].target_rpm = InputRPM;
}

/* 四电机 */
void Motor_Control_All(int16_t target)
{
    for(uint8_t i = MOTOR_ID_MIN; i <= MOTOR_ID_MAX; i++)
    {
        Motor_Speed_Control_Smooth(target, i);
    }
}

/* 急停 */
void Motor_Stop_Immediately(uint8_t ID)
{
    uint32_t primask;
    can_bus_status_t status;

    if(Motor_ID_Is_Valid(ID) == 0U)
    {
        return;
    }

    if(Motor_Driver_Initialized == 0U)
    {
        Motor_Driver_Init();
    }

    primask = Motor_Enter_Critical();
    MotorStates[ID - MOTOR_ID_MIN].target_rpm = 0;
    MotorStates[ID - MOTOR_ID_MIN].current_rpm = 0;
    Motor_Drive(0, ID);

    status = can_bus_send_std(&hfdcan1,
                              MOTOR_SPEED_COMMAND_CAN_ID,
                              Motor_TxData_0x32,
                              sizeof(Motor_TxData_0x32),
                              0U);
    Motor_Exit_Critical(primask);

    if(status != CAN_BUS_OK)
    {
        Motor_Smooth_Tx_Error_Count++;
    }
}

/* TIM中断 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM6)
    {
        Motor_Smooth_Tick_Count++;

        for(uint8_t i = 0U; i < MOTOR_DRIVE_COUNT; i++)
        {
            int16_t target_rpm = MotorStates[i].target_rpm;
            int16_t current_rpm = MotorStates[i].current_rpm;
            int32_t diff = (int32_t)target_rpm - (int32_t)current_rpm;

            if(diff > MotorStates[i].accel_step_rpm)
            {
                current_rpm += MotorStates[i].accel_step_rpm;
            }
            else if(diff < -MotorStates[i].accel_step_rpm)
            {
                current_rpm -= MotorStates[i].accel_step_rpm;
            }
            else
            {
                current_rpm = target_rpm;
            }

            MotorStates[i].current_rpm = current_rpm;
            Motor_Drive(current_rpm, (uint8_t)(i + MOTOR_ID_MIN));
        }

        /* 中断中禁止等待；FIFO 满则记错，下一拍（10 ms 后）自然重试。 */
        if(can_bus_send_std(&hfdcan1,
                            MOTOR_SPEED_COMMAND_CAN_ID,
                            Motor_TxData_0x32,
                            sizeof(Motor_TxData_0x32),
                            0U) != CAN_BUS_OK)
        {
            Motor_Smooth_Tx_Error_Count++;
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

    can_send(&hfdcan1, MOTOR_QUERY_CAN_ID, tx_buf, MOTOR_CAN_DATA_LENGTH);
}

/* 数据解析 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    /* 舵向电机使用 FDCAN2；其反馈不能写入 1~4 号驱动电机缓存。 */
    if(hfdcan == NULL || hfdcan->Instance != FDCAN1)
    {
        return;
    }

    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        FDCAN_RxHeaderTypeDef RxHeader;

        if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            uint8_t actual_id = Motor_Last_Query_ID;

            if(Motor_ID_Is_Valid(actual_id) != 0U)
            {
                Motor_Reporter_Cache[actual_id - MOTOR_ID_MIN].FBSpeed =
                (int16_t)((RxData[0] << 8) | RxData[1]);

                Motor_Reporter_Cache[actual_id - MOTOR_ID_MIN].Position =
                (uint16_t)((RxData[2] << 8) | RxData[3]);

                Motor_Reporter_Cache[actual_id - MOTOR_ID_MIN].ErrCode =
                RxData[4];
            }
        }
    }
}

/* 轮询读取 */
void App_Monitor_Read(void)
{
    /* 先保存本次真正发送的节点 ID，反馈回调不再依赖 query_id 的递增时序。 */
    Motor_Last_Query_ID = query_id;
    Ck_Check(query_id, 1, 4, 5, NULL);

    query_id++;
    if(query_id > MOTOR_ID_MAX)
        query_id = MOTOR_ID_MIN;
}
