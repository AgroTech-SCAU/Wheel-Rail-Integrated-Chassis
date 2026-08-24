#ifndef _chassis_service_h_
#define _chassis_service_h_

/**
 * @file chassis_service.h
 * @brief 轮轨复合底盘服务接口
 */

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 底盘服务调用状态码
 */
typedef enum {
    CHASSIS_SERVICE_STATUS_OK = 0,
    CHASSIS_SERVICE_STATUS_INVALID_PARAM,
    CHASSIS_SERVICE_STATUS_NOT_INITIALIZED,
    CHASSIS_SERVICE_STATUS_INIT_FAILED,
    CHASSIS_SERVICE_STATUS_DEVICE_ERROR,
    CHASSIS_SERVICE_STATUS_KINEMATICS_ERROR,
    CHASSIS_SERVICE_STATUS_FAULT_LATCHED,
    CHASSIS_SERVICE_STATUS_UNSAFE_STATE,
} ChassisServiceStatus;

/**
 * @brief 底盘锁存故障类型
 */
typedef enum {
    CHASSIS_FAULT_NONE = 0,
    CHASSIS_FAULT_INITIALIZATION,
    CHASSIS_FAULT_DRIVE_TRANSMIT,
    CHASSIS_FAULT_STEER_TRANSMIT,
    CHASSIS_FAULT_DRIVE_FEEDBACK_TIMEOUT,
    CHASSIS_FAULT_KINEMATICS,
    CHASSIS_FAULT_MANUAL_STOP,
} ChassisFault;

/**
 * @brief 底盘服务只读状态快照
 */
typedef struct {
    float command_vx;      /**< x 方向目标线速度 单位 m/s */
    float command_vy;      /**< y 方向目标线速度 单位 m/s */
    float command_wz;      /**< z 轴目标角速度 单位 rad/s */
    uint32_t update_count; /**< 控制更新计数 */
    ChassisFault fault;    /**< 当前锁存故障 */
    bool initialized;      /**< 服务是否初始化完成 */
    bool remote_online;    /**< 遥控链路是否在线 */
    bool remote_enabled;   /**< 遥控速度使能是否开启 */
    bool fault_latched;    /**< 是否存在锁存故障 */
} ChassisServiceState;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化底盘服务及全部依赖
 * @return ChassisServiceStatus 状态码
 */
ChassisServiceStatus chassis_service_init(void);
/**
 * @brief 执行一次底盘服务轮询
 * @return ChassisServiceStatus 状态码
 */
ChassisServiceStatus chassis_service_update(void);
/**
 * @brief 锁存人工停止故障并停止全部执行器
 * @return ChassisServiceStatus 状态码
 */
ChassisServiceStatus chassis_service_stop(void);
/**
 * @brief 在收到新的安全遥控帧后显式清除锁存故障
 * @return ChassisServiceStatus 状态码
 */
ChassisServiceStatus chassis_service_fault_clear(void);
/**
 * @brief 获取底盘服务状态快照
 * @param out 输出状态
 * @return ChassisServiceStatus 状态码
 */
ChassisServiceStatus chassis_service_get_state(ChassisServiceState* out);
/**
 * @brief 将服务状态码转换为静态字符串
 */
const char* chassis_service_status_str(ChassisServiceStatus status);
/**
 * @brief 将锁存故障转换为静态字符串
 */
const char* chassis_service_fault_str(ChassisFault fault);

#endif
