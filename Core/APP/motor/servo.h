#ifndef __SERVO_H___
#define __SERVO_H__

#include "main.h"
#include "fdcan.h" // 依赖你的 FDCAN 配置

/* ------------------ RS06 协议基础配置 ------------------ */
#define RS06_HOST_ID        0xFD   // 默认主机 ID
// 29位扩展帧 ID 计算宏： 通信类型(位24~28) | 主机ID(位8~15) | 目标电机ID(位0~7)
#define RS06_EXT_ID(type, host, motor) (((uint32_t)(type) << 16) | ((uint32_t)(host) << 8) | ((uint32_t)(motor)))

/* ------------------ 通信类型 (Type) ------------------ */
#define RS06_TYPE_RUN       0x0100   // 运控模式控制
#define RS06_TYPE_ENABLE    0x0300   // 使能电机
#define RS06_TYPE_STOP      0x0400   // 停止电机
#define RS06_TYPE_SET_ZERO  0x0600   // 设置机械零位
#define RS06_TYPE_SET_ID    0x0701   // 修改电机 ID
#define RS06_TYPE_WR_PARAM  0x1200   // 写入参数 (Type 18)

/* ------------------ 参数字典索引 ------------------ */
#define RS06_PARAM_RUN_MODE 0x7005    // 运行模式参数地址 
#define RS06_PARAM_LOC_REF  0x7016    // 【重要修正】位置模式期望位置 (Float) 
#define RS06_PARAM_LIMIT_SPD 0x7017   // 位置模式速度限制 (Float) 
#define RS06_TYPE_SAVE      0x1600   // 通信类型 22: 电机数据保存帧

/* 模式定义  */
#define RS06_MODE_MIT         0   // 运控模式
#define RS06_MODE_PP          1   // 位置模式 (PP)
#define RS06_MODE_CSP         5   // 位置模式 (CSP)

/* ------------------ 运控模式物理量限幅 ------------------ */
// 严格遵守说明书的浮点数转换范围 
#define RS06_P_MIN         -12.57f // 最小角度 (rad)
#define RS06_P_MAX          12.57f // 最大角度 (rad)
#define RS06_V_MIN         -50.0f  // 最小速度 (rad/s)
#define RS06_V_MAX          50.0f  // 最大速度 (rad/s)
#define RS06_KP_MIN         0.0f   // 最小 Kp
#define RS06_KP_MAX         5000.0f// 最大 Kp 
#define RS06_KD_MIN         0.0f   // 最小 Kd
#define RS06_KD_MAX         100.0f // 最大 Kd 

/* ------------------ 模块化函数声明 ------------------ */
void RS06_Enable(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id);
void RS06_Stop(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id);
void RS06_Change_ID(FDCAN_HandleTypeDef* hfdcan, uint8_t old_id, uint8_t new_id);
void RS06_Set_Mode(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id, uint8_t mode);
void RS06_Set_Position_Target(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id, float angle_rad);
void RS06_Set_Position(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id, float angle_rad, float speed_rad_s, float kp, float kd, float t_ff);
// void RS06_Set_Position_Instruction10(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id, float angle_rad, float speed_rad_s);
void RS06_reset(void);
void RS06_Set_Mechanical_Zero(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id);
void RS06_Save_Config(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id);
void RS06_Zeroing_And_Save_Process(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id);

#endif /* __RS06_H__ */