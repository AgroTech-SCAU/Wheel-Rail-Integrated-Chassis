    #ifndef __MOTOR_H
    #define __MOTOR_H

    #include "main.h"

    #define SPEED_RPM_MAX   21000
    #define SPEED_RPM_MIN  -21000

    extern TIM_HandleTypeDef htim6;

    typedef struct {
        int16_t FBSpeed;
        int16_t ECurru;
        int16_t Position;
        uint8_t ErrCode;
        uint8_t FBMode;
    } reporter;

    /* 初始化 */
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

    // 修复3：将 uint16_t 改为有符号 int16_t，否则无法反转
    void Motor_Control_All(int16_t target); 

    /* 急停 */
    void Motor_Stop_Immediately(uint8_t ID);

    /* 回调 */
    void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
    void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

    /* 数据读取 */
    void App_Monitor_Read(void);

    #endif