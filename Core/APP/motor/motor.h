#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h" // 包含 HAL 库相关定义

/* --- 宏定义 --- */
#define SPEED_RPM_MAX   210
#define SPEED_RPM_MIN  -210

extern TIM_HandleTypeDef htim4; 
extern TIM_HandleTypeDef htim6;

/* --- 结构体定义 --- */

/**
 * @brief 电机反馈数据解析结构体
 */
typedef struct {
    int16_t FBSpeed;   // 反馈转速
    int16_t ECurru;    // 反馈电流
    int16_t Position;  // 反馈位置
    uint8_t ErrCode;   // 错误代码
    uint8_t FBMode;    // 当前模式
} reporter;

/* --- 函数原型声明 --- */

/**
 * @brief 电机驱动及平滑算法初始化
 * @note 内部会自动启动定时器中断 (TIM6)
 */
void Motor_Driver_Init(void);

/**
 * @brief 设置电机反馈模式
 * @param FeedBack: 反馈指令
 * @param ID: 电机ID (1-8)
 */
uint8_t Motor_Set_FeedBack(unsigned char FeedBack, uint8_t ID);

/**
 * @brief 电机位置校准模式
 */
void Motor_Calibration(void);

/**
 * @brief 检查并解析电机反馈
 */
void Ck_Check(uint8_t ID, uint8_t Check1, uint8_t Check2, uint8_t Check3, reporter* reporter);

/**
 * @brief 直接解析报告模式数据
 */
void Obtain_Motor_Report(reporter* report);

/**
 * @brief 修改电机 ID (需单独连接电机时修改)
 */
uint8_t ID_Set(uint8_t ID);

/**
 * @brief 修改电机运行模式
 */
uint8_t Motor_SetMode(unsigned char Mode);

/**
 * @brief 应用层普通控制函数 (直接改变速度，可能会有抖动)
 */
void Motor_Speed_Control(int16_t InputRPM, uint8_t ID);


/* --- 算法优化部分接口 --- */

/**
 * @brief 应用层平滑控制函数 (推荐使用)
 * @param InputRPM: 目标转速 (-210 到 210)
 * @param ID: 电机ID (1-4)
 */
void Motor_Speed_Control_Smooth(int16_t InputRPM, uint8_t ID);

/**
 * @brief 电机紧急停止 (强制清零转速，跳过平滑斜坡)
 */
void Motor_Stop_Immediately(uint8_t ID);

/**
 * @brief 定时器回调函数声明 (通常由 HAL 库内部调用，在此声明以防链接问题)
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

/*应用层调用函数，四个电机*/
void Motor_Control_All(uint16_t target);

/*串口读取电机数据*/
void App_Monitor_Read(void);

#endif /* __MOTOR_H */