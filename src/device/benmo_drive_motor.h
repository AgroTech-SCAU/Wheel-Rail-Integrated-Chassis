#ifndef _benmo_drive_motor_h_
#define _benmo_drive_motor_h_

/**
 * @file benmo_drive_motor.h
 * @brief 本末驱动电机协议驱动接口
 */

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

#define BENMO_DRIVE_MOTOR_COUNT 4u
#define BENMO_DRIVE_MOTOR_ID_MIN 1u
#define BENMO_DRIVE_MOTOR_ID_MAX 4u
#define BENMO_DRIVE_MOTOR_SPEED_COMMAND_CAN_ID 0x032u
#define BENMO_DRIVE_MOTOR_QUERY_CAN_ID 0x107u
#define BENMO_DRIVE_MOTOR_CAN_DATA_LEN 8u
#define BENMO_DRIVE_MOTOR_RPM_MIN (-210)
#define BENMO_DRIVE_MOTOR_RPM_MAX 210
#define BENMO_DRIVE_MOTOR_RAMP_STEP_RPM 5

/**
 * @brief 本末驱动电机状态码
 */
typedef enum {
    BENMO_DRIVE_STATUS_OK = 0,
    BENMO_DRIVE_STATUS_INVALID_PARAM,
    BENMO_DRIVE_STATUS_NOT_INITIALIZED,
    BENMO_DRIVE_STATUS_PORT_ERROR,
    BENMO_DRIVE_STATUS_FEEDBACK_TIMEOUT,
} BenmoDriveMotorStatus;

/**
 * @brief 本末驱动电机平台能力接口
 */
typedef struct {
    bool (*send_standard)(uint32_t id, const uint8_t* data, uint8_t len);
    uint32_t (*now_ms)(void);
    uint32_t (*critical_enter)(void);
    void (*critical_exit)(uint32_t state);
} BenmoDriveMotorPortOps;

/**
 * @brief 本末驱动电机初始化配置
 */
typedef struct {
    const BenmoDriveMotorPortOps* ops;
    uint32_t feedback_timeout_ms;
} BenmoDriveMotorConfig;

/**
 * @brief 单个驱动电机反馈数据
 */
typedef struct {
    int16_t speed_rpm;          /**< 反馈转速 单位 RPM */
    int16_t current_raw;        /**< 协议保留的原始电流字段 */
    uint16_t position_raw;      /**< 协议原始位置字段 */
    uint8_t error_code;         /**< 电机错误码 */
    uint8_t mode;               /**< 电机反馈模式 */
    uint32_t last_update_ms;    /**< 最近反馈时间戳 */
    uint32_t response_can_id;   /**< 首次绑定的响应 CAN ID */
    bool response_can_id_valid; /**< 响应 CAN ID 是否已经绑定 */
    bool valid;                 /**< 是否已经接收有效反馈 */
} BenmoDriveMotorFeedback;

/**
 * @brief 单个驱动电机控制与反馈节点
 */
typedef struct {
    int16_t target_rpm;
    int16_t current_rpm;
    BenmoDriveMotorFeedback feedback;
} BenmoDriveMotorNode;

/**
 * @brief 四节点本末驱动电机实例
 */
typedef struct {
    const BenmoDriveMotorPortOps* ops;
    BenmoDriveMotorNode nodes[BENMO_DRIVE_MOTOR_COUNT];
    uint8_t command_data[BENMO_DRIVE_MOTOR_CAN_DATA_LEN];
    uint8_t next_query_id;
    uint8_t last_query_id;
    uint32_t initialized_at_ms;
    uint32_t feedback_timeout_ms;
    uint32_t tx_error_count;
    bool awaiting_feedback;
    bool initialized;
} BenmoDriveMotor;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化本末驱动电机实例
 * @param self 驱动实例
 * @param config 初始化配置
 * @return BenmoDriveMotorStatus 状态码
 */
BenmoDriveMotorStatus benmo_drive_motor_init(BenmoDriveMotor* self,
                                             const BenmoDriveMotorConfig* config);
/**
 * @brief 设置单个节点平滑目标转速
 */
BenmoDriveMotorStatus benmo_drive_motor_set_target_rpm(BenmoDriveMotor* self,
                                                       uint8_t motor_id,
                                                       int16_t target_rpm);
/**
 * @brief 执行一次四节点速度斜坡与组合帧发送
 */
BenmoDriveMotorStatus benmo_drive_motor_update(BenmoDriveMotor* self);
/**
 * @brief 立即清零全部节点并发送零速组合帧
 */
BenmoDriveMotorStatus benmo_drive_motor_stop_all(BenmoDriveMotor* self);
/**
 * @brief 请求下一个节点反馈
 */
BenmoDriveMotorStatus benmo_drive_motor_request_next_feedback(BenmoDriveMotor* self);
/**
 * @brief 解析当前等待节点的反馈帧
 */
BenmoDriveMotorStatus benmo_drive_motor_handle_feedback(BenmoDriveMotor* self,
                                                        uint32_t response_can_id,
                                                        const uint8_t* data,
                                                        uint8_t len);
/**
 * @brief 检查全部节点反馈是否超时
 */
BenmoDriveMotorStatus benmo_drive_motor_check_feedback(const BenmoDriveMotor* self);
/**
 * @brief 重新开始反馈超时观察窗口
 */
BenmoDriveMotorStatus benmo_drive_motor_reset_feedback_monitor(BenmoDriveMotor* self);
/**
 * @brief 获取指定节点反馈快照
 */
BenmoDriveMotorStatus benmo_drive_motor_get_feedback(const BenmoDriveMotor* self,
                                                     uint8_t motor_id,
                                                     BenmoDriveMotorFeedback* out);

#endif
