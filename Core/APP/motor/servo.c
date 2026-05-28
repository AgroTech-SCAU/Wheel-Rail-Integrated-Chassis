#include "servo.h"
#include "can.h"
#include "main.h"
  #include <stdio.h>
  #include <string.h>


extern void can_send(FDCAN_HandleTypeDef* hfdcanx, uint32_t id, uint8_t* data, uint8_t len);

/**
 * @brief  浮点数线性映射转 16位无符号整数 (运控模式核心算法)
 */
static uint16_t float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x - x_min;
    
    // 限幅保护，防止溢出
    if(offset < 0) offset = 0;
    if(offset > span) offset = span;
    
    return (uint16_t)((offset / span) * (float)((1 << bits) - 1));
}

/**
 * @brief  1. 使能电机
 * @param  hfdcan: CAN 句柄
 * @param  motor_id: 目标电机 ID
 */
void RS06_Enable(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id) {
    uint32_t id = RS06_EXT_ID(RS06_TYPE_ENABLE, RS06_HOST_ID, motor_id);
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // 数据域全0
    can_send(hfdcan, id, data, 8);
}

/**
 * @brief  2. 停止/卸载电机
 */
void RS06_Stop(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id) {
    uint32_t id = RS06_EXT_ID(RS06_TYPE_STOP, RS06_HOST_ID, motor_id);
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    can_send(hfdcan, id, data, 8);
}

/**
 * @brief  3. 修改电机 ID (需重启生效) [cite: 287]
 */
void RS06_Change_ID(FDCAN_HandleTypeDef* hfdcan, uint8_t old_id, uint8_t new_id) {
    uint32_t id = RS06_EXT_ID(RS06_TYPE_SET_ID, new_id, old_id);
    uint8_t data[8] = {0}; 
    can_send(hfdcan, id, data, 8);
}

/**
 * @brief  通信类型 6：设置电机机械零位[cite: 1]
 * @param  motor_id: 目标电机 ID
 * @note   该指令会将当前电机的机械位置强制清零。
 */
void RS06_Set_Mechanical_Zero(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id) {
    uint32_t id = RS06_EXT_ID(RS06_TYPE_SET_ZERO, RS06_HOST_ID, motor_id);
    
    // 根据手册：Byte[0] = 1[cite: 1]
    uint8_t data[8] = {1, 0, 0, 0, 0, 0, 0, 0}; 
    
    can_send(hfdcan, id, data, 8);
}

/**
 * @brief  通信类型 22：电机数据保存帧[cite: 1]
 * @param  motor_id: 目标电机 ID
 * @note   必须发送此帧，设置的零位（MechOffset）才会掉电不丢失。
 */
void RS06_Save_Config(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id) {
    uint32_t id = RS06_EXT_ID(RS06_TYPE_SAVE, RS06_HOST_ID, motor_id);
    
    // 根据手册 Type 22 字节序列要求[cite: 1]
    // 对应数据区 1 内容：01 02 03 04 05 06 07 08 (手册示例序列)
    uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    
    can_send(hfdcan, id, data, 8);
}

/**
 * @brief 完整标零保存流程
 */
void RS06_Zeroing_And_Save_Process(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id) {
    // 1. 设置机械零点
    RS06_Set_Mechanical_Zero(hfdcan, motor_id);
    
    // 2. 适当延时（手册建议等待反馈或留出 Flash 写入缓冲时间）
    HAL_Delay(100); 
    
    // 3. 执行永久保存指令
    RS06_Save_Config(hfdcan, motor_id);
    
    // 4. 再次延时确保保存成功
    HAL_Delay(500); 
}



/**
 * @brief  5. 运控模式指令：指定角度转动
 * @note   严格遵循说明书截图中的 16-12-12-12-12 位压缩格式
 * @param  hfdcan:      FDCAN 句柄
 * @param  motor_id:    目标电机 ID
 * @param  angle_rad:   目标位置 (P_des), 范围 RS06_P_MIN ~ RS06_P_MAX
 * @param  speed_rad_s: 目标速度 (V_des), 范围 RS06_V_MIN ~ RS06_V_MAX
 * @param  kp:          位置比例系数 (Kp), 范围 RS06_KP_MIN ~ RS06_KP_MAX
 * @param  kd:          速度比例系数 (Kd), 范围 RS06_KD_MIN ~ RS06_KD_MAX
 * @param  t_ff:        前馈扭矩 (T_ff), 建议范围 -36.0 到 36.0 (Nm)
 */
void RS06_Set_Position(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id, float angle_rad, float speed_rad_s, float kp, float kd, float t_ff) {
    
    // 1. 根据你的宏定义计算 ID: (0x0100 << 16) | (0xFD << 8) | motor_id
    uint32_t id = RS06_EXT_ID(RS06_TYPE_RUN, RS06_HOST_ID, motor_id);
    
    // 2. 将浮点数转换为特定位宽的无符号整数 (线性映射)
    // 严格对应说明书：P(16bit), V(12bit), KP(12bit), KD(12bit), T(12bit)
    uint16_t p_int  = float_to_uint(angle_rad, RS06_P_MIN,  RS06_P_MAX,  16);
    uint16_t v_int  = float_to_uint(speed_rad_s, RS06_V_MIN,  RS06_V_MAX,  12);
    uint16_t kp_int = float_to_uint(kp,        RS06_KP_MIN, RS06_KP_MAX, 12);
    uint16_t kd_int = float_to_uint(kd,        RS06_KD_MIN, RS06_KD_MAX, 12);
    uint16_t t_int  = float_to_uint(t_ff,      -36.0f,      36.0f,       12);
    
    uint8_t data[8];
    
    // 3. 严格按照 MIT 协议位域拼接逻辑填充 8 字节数据域
    // Byte 0, 1: P_des (16位)
    data[0] = (p_int >> 8) & 0xFF;                  // P 高 8 位
    data[1] = p_int & 0xFF;                         // P 低 8 位
    
    // Byte 2, 3: V_des (12位) + Kp (高 4 位)
    data[2] = (v_int >> 4) & 0xFF;                  // V 高 8 位
    data[3] = ((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F); // V 低 4 位 + Kp 高 4 位
    
    // Byte 4, 5: Kp (低 8 位) + Kd (高 8 位)
    data[4] = kp_int & 0xFF;                        // Kp 低 8 位
    data[5] = (kd_int >> 4) & 0xFF;                 // Kd 高 8 位
    
    // Byte 6, 7: Kd (低 4 位) + T_ff (12位)
    data[6] = ((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F); // Kd 低 4 位 + T 高 4 位
    data[7] = t_int & 0xFF;                         // T 低 8 位
    
    // 4. 发送数据
    // 由于 id = 0x0100FDxx > 0x7FF，你底层的 can_send 会自动选择 FDCAN_EXTENDED_ID
    can_send(hfdcan, id, data, 8);
}

/**
 * @brief  4. 设置电机运行模式 (通信类型 18 写入 0x7005) [cite: 290, 298]
 */
void RS06_Set_Mode(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id, uint8_t mode) {
    uint32_t id = RS06_EXT_ID(RS06_TYPE_WR_PARAM, RS06_HOST_ID, motor_id);
    uint8_t data[8] = {0};

    // 索引 0x7005 (小端)
    data[0] = RS06_PARAM_RUN_MODE & 0xFF; 
    data[1] = (RS06_PARAM_RUN_MODE >> 8) & 0xFF;
    
    // 数据域 (小端)
    data[4] = mode;

    can_send(hfdcan, id, data, 8);
}

/**
 * @brief  5. 发送绝对位置指令 (通信类型 18 写入 0x7016) 
 * @note   舵轮控制核心函数！适用 PP 和 CSP 模式。
 */
void RS06_Set_Position_Target(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id, float angle_rad) {
    uint32_t id = RS06_EXT_ID(RS06_TYPE_WR_PARAM, RS06_HOST_ID, motor_id);
    uint8_t data[8] = {0};

    // 索引 0x7016 (小端)
    data[0] = RS06_PARAM_LOC_REF & 0xFF; 
    data[1] = (RS06_PARAM_LOC_REF >> 8) & 0xFF;
    
    // 拷贝 32位 float 内存数据到 CAN 帧中
    memcpy(&data[4], &angle_rad, 4);

    can_send(hfdcan, id, data, 8);
}

/**
 * @brief  7. 舵轮测试流程
 * 给出不污染主分支的测试函数，方便你在开发过程中验证 RS06 的基本功能是否正常。
 */
void RS06_reset(void){
uint8_t motor_ids[] = {0x05, 0x06, 0x07, 0x08};
    float targets[] = {0.00f, 0.00f, 0.00f, 0.00f};
    int motor_count = sizeof(motor_ids) / sizeof(motor_ids[0]);

    // 第一步：批量初始化和使能
    for (int i = 0; i < motor_count; i++) {
        RS06_Set_Mode(&hfdcan2, motor_ids[i], RS06_MODE_PP);
        
        RS06_Enable(&hfdcan2, motor_ids[i]);
    }

    // 第二步：批量发送位置指令
    for (int i = 0; i < motor_count; i++) {
        RS06_Set_Position_Target(&hfdcan2, motor_ids[i], targets[i]);
    }

    // 第三步：等待电机运动完成
    HAL_Delay(1000);

    // 第四步：批量停止
    for (int i = 0; i < motor_count; i++) {
        RS06_Stop(&hfdcan2, motor_ids[i]);
    }

}


