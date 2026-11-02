#ifndef _chassis_service_h_
#define _chassis_service_h_

/**
 * @file chassis_service.h
 * @brief 轮轨复合底盘服务接口
 */

#include <stdbool.h>
#include <stdint.h>

#include "steer_wheel_kinematics.h"
#include "wr_can_protocol.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 底盘运动学初始化配置
 * @param model 底盘几何参数和单轮速度上限
 * @param yaw_bias 平移偏航前馈补偿参数，在逆运动学中使用
 */
typedef struct {
    SteerWheelModel model;
    SteerWheelYawBiasConfig yaw_bias;
    float rail_radius_m;       /**< 轨轮有效半径；0 表示未标定，拒绝 ON_RAIL */
    int8_t drive_direction[4]; /**< FL/FR/RR/RL 原始电机方向，必须为 +1/-1 */
    int8_t steer_direction[4]; /**< RS06 5/6/7/8 原始电机方向，必须为 +1/-1 */
    int8_t probe_direction[4]; /**< 探轨时各舵向的车体坐标方向；0 表示未标定 */
    float probe_min_angle[4];  /**< 各轮探轨车体坐标最小机械角；等于最大值表示未标定 */
    float probe_max_angle[4];  /**< 各轮探轨车体坐标最大机械角 */
    float probe_hard_torque_nm; /**< 探轨硬停转矩；0 表示未标定，拒绝 PROBE */
    float probe_max_speed_rad_s; /**< 探轨最大角速度；0 表示未标定 */
    float probe_max_temperature_c; /**< 探轨硬停温度；0 表示未标定 */
    bool require_estop;        /**< 为 true 时，急停输入无效视为不安全 */
    bool legacy_steer_57_correction; /**< ID5/7 轮组的安装几何修正 */
} ChassisServiceConfig;

typedef struct {
    bool physical_estop;
    bool estop_valid;
    bool battery_valid;
    float battery_voltage;
    float battery_current;
} ChassisBoardInputs;

typedef struct {
    WrChassisState chassis;
    WrModuleState modules[4];
    WrSteeringTaskState task;
    uint8_t active_module_mask;
} ChassisServiceSnapshot;

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
    CHASSIS_FAULT_STEER_FEEDBACK_TIMEOUT,
    CHASSIS_FAULT_STEER_ANGLE_LIMIT,
    CHASSIS_FAULT_DRIVE_FEEDBACK_TIMEOUT,
    CHASSIS_FAULT_KINEMATICS,
    CHASSIS_FAULT_MANUAL_STOP,
    CHASSIS_FAULT_DRIVE_HARDWARE,
    CHASSIS_FAULT_BOARD_ESTOP,
    CHASSIS_FAULT_TASK_SAFETY,
    CHASSIS_FAULT_STEER_HARDWARE,
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
    uint8_t source;        /**< WR_SOURCE_* */
    uint8_t mode;          /**< WR_MODE_* */
    bool enabled;
    bool command_timed_out;
    bool motor_power_ready;
    uint32_t last_command_sequence;
} ChassisServiceState;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化底盘服务及全部依赖
 * @return ChassisServiceStatus 状态码
 */
ChassisServiceStatus chassis_service_init(void);
ChassisServiceConfig chassis_service_default_config(void);
/**
 * @brief 使用指定运动学配置初始化底盘服务及全部依赖
 * @param config 配置指针；初始化时复制参数，无需长期保留
 * @return ChassisServiceStatus 状态码
 */
ChassisServiceStatus chassis_service_init_with_config(
    const ChassisServiceConfig* config);
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
 * @brief 在板级安全条件满足后显式清故障；遥控在线时还需故障后的新禁能帧
 * @return ChassisServiceStatus 状态码
 */
ChassisServiceStatus chassis_service_fault_clear(void);
/**
 * @brief 获取底盘服务状态快照
 * @param out 输出状态
 * @return ChassisServiceStatus 状态码
 */
ChassisServiceStatus chassis_service_get_state(ChassisServiceState* out);
void chassis_service_set_board_inputs(const ChassisBoardInputs* inputs);
WrResult chassis_service_apply_control(const WrControlCommand* command,
                                       uint32_t sequence, uint64_t rx_us);
WrResult chassis_service_apply_velocity(const WrVelocityCommand* command,
                                        uint32_t sequence, uint64_t rx_us);
WrResult chassis_service_apply_steering_task(const WrSteeringTaskCommand* command,
                                             uint32_t sequence, uint64_t rx_us);
ChassisServiceStatus chassis_service_get_snapshot(ChassisServiceSnapshot* out,
                                                  uint64_t now_us);
/**
 * @brief 将服务状态码转换为静态字符串
 */
const char* chassis_service_status_str(ChassisServiceStatus status);
/**
 * @brief 将锁存故障转换为静态字符串
 */
const char* chassis_service_fault_str(ChassisFault fault);

#endif
