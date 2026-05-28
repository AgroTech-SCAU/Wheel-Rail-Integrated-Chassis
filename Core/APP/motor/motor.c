#include "motor.h"
#include "fdcan.h"
#include <string.h>
#include "can.h"
#include "main.h"
#include "tim.h"

// 引用外部句柄
extern FDCAN_HandleTypeDef hfdcan1; 
extern TIM_HandleTypeDef htim6; 
extern reporter Motor_Reporter_Data; 
extern reporter Motor_Reporter_Cache[4]; 
extern uint8_t query_id;
// 数据缓冲区
static uint8_t Motor_TxData_0x32[8] = {0}; // 用于缓存 ID 1-4 的速度指令
static uint8_t Motor_TxData_0x33[8] = {0}; // 用于缓存 ID 5-8 的速度指令
uint8_t RxData[8];
static uint8_t Temp;

#define SPEED_RPM_MAX  210
#define SPEED_RPM_MIN -210

/* --- 平滑控制结构体 --- */
typedef struct {
    int16_t target_rpm;    // 用户设定的目标速度
    float   current_rpm;   // 当前平滑过后的实时速度 (用float保证精度)
    float   accel_step;    // 步进值，决定加速快慢 (RPM/10ms)
} Motor_Smooth_Ctrl_t;

// 对应ID 1-4 的电机状态
static Motor_Smooth_Ctrl_t MotorStates[4];

/**
 * @brief 初始化电机驱动及平滑控制参数
 */
void Motor_Driver_Init(void) {
    HAL_GPIO_WritePin(CAN1_EN_GPIO_Port, CAN1_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CAN2_EN_GPIO_Port, CAN2_EN_Pin, GPIO_PIN_SET);

    // FDCAN 过滤器配置 (保持原样)
    if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                                    FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) {
        Error_Handler();
    }
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    // FDCAN 过滤器配置 (保持原样)
    if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                                    FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) {
        Error_Handler();
    }
    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    // 初始化平滑控制参数
    for(int i=0; i<4; i++) {
        MotorStates[i].target_rpm = 0;
        MotorStates[i].current_rpm = 0.0f;
        MotorStates[i].accel_step = 0.4f;  
    }

    HAL_TIM_Base_Start_IT(&htim6);
   
}

// //优化前
// void Motor_Driver_Init(void) {
//     HAL_GPIO_WritePin(CAN1_EN_GPIO_Port, CAN1_EN_Pin, GPIO_PIN_SET);
//     HAL_GPIO_WritePin(CAN2_EN_GPIO_Port, CAN2_EN_Pin, GPIO_PIN_SET);

//     if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
//         FDCAN_ACCEPT_IN_RX_FIFO0,
//         FDCAN_ACCEPT_IN_RX_FIFO0,
//         FDCAN_REJECT_REMOTE,
//         FDCAN_REJECT_REMOTE) != HAL_OK) {
//         Error_Handler();
//     }
//     if(HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
//         Error_Handler();
//     }
//     if(HAL_FDCAN_ActivateNotification(&hfdcan1,
//         FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
//         0) != HAL_OK) {
//         Error_Handler();
//     }

//     HAL_Delay(1500);
//     // dm_motor_init();
// }


/**
 * @brief 设置反馈模式
 * @param ID: 1-8
 * @param FeedBack: 反馈指令
 */
uint8_t Motor_Set_FeedBack(unsigned char FeedBack, uint8_t ID)
{
    memset(Motor_TxData_0x32, 0, 8);
    
    if(ID > 0 && ID <= 8) {
        Motor_TxData_0x32[ID-1] = FeedBack;
    }
    
    can_send(&hfdcan1, 0x106, Motor_TxData_0x32, 8);
    
    return RxData[0]; // 注意：此处仍为上一帧数据，建议在中断中处理接收
}

/**
 * @brief 电机校准模式
 */
void Motor_Calibration(void)
{
    memset(Motor_TxData_0x32, 0, 8);
    can_send(&hfdcan1, 0x104, Motor_TxData_0x32, 8);
}

/**
 * @brief 仅负责发送查询指令，不解析数据
 */
void Ck_Check(uint8_t ID, uint8_t Check1, uint8_t Check2, uint8_t Check3, reporter* unused)
{
    uint8_t tx_buf[8];
    tx_buf[0] = ID;
    tx_buf[1] = Check1;
    tx_buf[2] = Check2;
    tx_buf[3] = Check3;
    tx_buf[4] = 0xAA;
    tx_buf[5] = 0; tx_buf[6] = 0; tx_buf[7] = 0;

    can_send(&hfdcan1, 0x107, tx_buf, 8); 
}
/**
 * @brief 报告模式解析
 */
void Obtain_Motor_Report(reporter* report)
{
    report->FBSpeed  = (RxData[0]<<8) + RxData[1];
    report->ECurru   = (RxData[2]<<8) + RxData[3];
    report->Position = (RxData[4]<<8) + RxData[5];
    report->ErrCode  = RxData[6];
    report->FBMode   = RxData[7];
}

/**
 * @brief 修改电机 ID(只能修改一个id)
 */
uint8_t ID_Set(uint8_t ID)
{
    memset(Motor_TxData_0x32, 0, 8);
    Motor_TxData_0x32[0] = ID;       
    can_send(&hfdcan1, 0x108, Motor_TxData_0x32, 8);
    return RxData[0];
}

/**
 * @brief 修改电机模式
 */
uint8_t Motor_SetMode(unsigned char Mode)
{
    memset(Motor_TxData_0x32, 0, 8);
    Motor_TxData_0x32[0] = Mode;
    can_send(&hfdcan1, 0x105, Motor_TxData_0x32, 8);
    return RxData[0];
}

/**
 * @brief 设置电机速度
 * @param ID: 1-4 (根据 0x32 的映射逻辑)
 */
// 底层驱动发送函数


extern FDCAN_HandleTypeDef hfdcan1;

void Motor_Drive(int16_t InputRPM, uint8_t ID)
{
   int32_t target_val = (int32_t)InputRPM * 100;
    
    // 限制在说明书允许的给定值范围内: -32767 ~ 32767
    if (target_val > 32767)  target_val = 32767;
    if (target_val < -32767) target_val = -32767;
    
    int16_t ScaledSpeed = (int16_t)target_val;

    // 处理 ID 1-4 (对应标识符 0x32)
    if (ID >= 1 && ID <= 4) 
    {
        // 计算数组索引：ID=1 -> [0,1], ID=2 -> [2,3]...
        uint8_t idx_high = (ID - 1) * 2;
        uint8_t idx_low  = (ID - 1) * 2 + 1;

        // 更新该电机的缓存数据 (大端模式：高位在前)
        Motor_TxData_0x32[idx_high] = (uint8_t)(ScaledSpeed >> 8);
        Motor_TxData_0x32[idx_low]  = (uint8_t)(ScaledSpeed & 0x00FF);

        // 发送完整的 8 字节数据，包含 1-4 号电机的最新状态
        can_send(&hfdcan1, 0x032, Motor_TxData_0x32, 8);
    }
    // 处理 ID 5-8 (对应标识符 0x33)
    else if (ID >= 5 && ID <= 8) 
    {
        // 计算数组索引：ID=5 -> [0,1], ID=6 -> [2,3]...
        uint8_t idx_high = (ID - 5) * 2;
        uint8_t idx_low  = (ID - 5) * 2 + 1;

        // 更新该电机的缓存数据 (大端模式：高位在前)
        Motor_TxData_0x33[idx_high] = (uint8_t)(ScaledSpeed >> 8);
        Motor_TxData_0x33[idx_low]  = (uint8_t)(ScaledSpeed & 0x00FF);

        // 发送完整的 8 字节数据，包含 5-8 号电机的最新状态
        can_send(&hfdcan1, 0x033, Motor_TxData_0x33, 8);
    }
}

/*
    应用层调用
*/

// 应用层速度控制函数
// 应用层速度控制函数
void Motor_Speed_Control(int16_t InputRPM, uint8_t ID)
{
    // 1. 范围限制 (Clamping)
    // 文档限制范围 -210 到 210 RPM 
    if (InputRPM > SPEED_RPM_MAX) {
        InputRPM = SPEED_RPM_MAX;
    } else if (InputRPM < SPEED_RPM_MIN) {
        InputRPM = SPEED_RPM_MIN;
    }

    // 2. 单位换算
    int16_t SendValue = InputRPM * 100;

    // 3. 发送驱动指令
    Motor_Drive(SendValue, ID);
}

/*
算法优化后

*/
/**
 * @brief 应用层调用：设定目标速度（非阻塞，不直接发CAN）
 */
void Motor_Speed_Control_Smooth(int16_t InputRPM, uint8_t ID)
{
    if (ID < 1 || ID > 4) return;

    // 限幅
    if (InputRPM > SPEED_RPM_MAX) InputRPM = SPEED_RPM_MAX;
    if (InputRPM < SPEED_RPM_MIN) InputRPM = SPEED_RPM_MIN;

    // 更新目标，真正的CAN发送由定时器中断接管
    MotorStates[ID-1].target_rpm = InputRPM;
}

void Motor_Control_All(uint16_t target) {
uint8_t motor_ids[] = {0x01, 0x02, 0x03, 0x04};
int motor_count = sizeof(motor_ids) / sizeof(motor_ids[0]);

    for (uint8_t i = 0; i < motor_count; i++) {
        Motor_Speed_Control_Smooth(target, motor_ids[i]);
    }
}

    



/**
 * @brief 定时器中断回调函数 (由HAL库自动调用)
 * @note 
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) 
    {
        for (int i = 0; i < 4; i++) 
        {
            // 1. 斜坡算法计算
            float diff = (float)MotorStates[i].target_rpm - MotorStates[i].current_rpm;
            
            if (diff > MotorStates[i].accel_step) {
                MotorStates[i].current_rpm += MotorStates[i].accel_step;
            } else if (diff < -MotorStates[i].accel_step) {
                MotorStates[i].current_rpm -= MotorStates[i].accel_step;
            } else {
                MotorStates[i].current_rpm = (float)MotorStates[i].target_rpm;
            }

            // 2. 将计算后的实时速度转换为发送值 (RPM * 100)
            int16_t send_val = (int16_t)(MotorStates[i].current_rpm * 100.0f);

            // 3. 填充缓存 (大端)
            Motor_TxData_0x32[i * 2]     = (uint8_t)(send_val >> 8);
            Motor_TxData_0x32[i * 2 + 1] = (uint8_t)(send_val & 0xFF);
        }

        // 4. 固定频率发送指令，控制4路电机
        can_send(&hfdcan1, 0x032, Motor_TxData_0x32, 8);
    }
    if(htim->Instance == TIM15){
        App_Monitor_Read();
    }
    
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        FDCAN_RxHeaderTypeDef RxHeader;
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
            if (query_id >= 1 && query_id <= 4) {
                // 对应 Ck_Check(id, 1, 4, 5) 的返回顺序：
                // RxData[0,1] -> Check1 (速度)
                // RxData[2,3] -> Check2 (位置)
                // RxData[4,5] -> Check3 (故障)
                Motor_Reporter_Cache[query_id - 1].FBSpeed  = (int16_t)((RxData[0] << 8) | RxData[1]);
                Motor_Reporter_Cache[query_id - 1].Position = (uint16_t)((RxData[2] << 8) | RxData[3]);
                Motor_Reporter_Cache[query_id - 1].ErrCode  = (uint8_t)((RxData[4] << 8) | RxData[5]);
            }
        }
    }
}


/*急停*/
void Motor_Stop_Immediately(uint8_t ID) {
    MotorStates[ID-1].target_rpm = 0;
    MotorStates[ID-1].current_rpm = 0; // 强制清零，跳过斜坡
}


reporter Motor_Reporter_Cache[4];
uint8_t query_id;

void App_Monitor_Read(void) {
    // 发送查询：1-速度(FBSpeed), 4-位置(Position), 5-故障码(ErrCode)
    Ck_Check(query_id, 1, 4, 5, NULL); 

    // 轮询 ID
    if (++query_id > 4) query_id = 1;
}
