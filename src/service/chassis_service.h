#ifndef _chassis_service_h_
#define _chassis_service_h_

/**
 * @file chassis_service.h
 * @brief 轮轨复合底盘服务接口
 * @note 上层建议包含 wheel_rail_sdk.h；本接口为单实例、串行轮询模型。
 *       配置和输出结构体由调用者提供；不动态分配，不持有调用者配置指针。
 */

#include <stdbool.h>
#include <stdint.h>

#include "steer_wheel_kinematics.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/** @brief 控制来源在初始化时确定，运行中不自动切换。 */
typedef enum {
    CHASSIS_CONTROL_REMOTE = 0, /**< 默认：保留 iBUS 遥控和 VRB/SWB/SWC 行为 */
    CHASSIS_CONTROL_EXTERNAL,   /**< 上层速度指令，不依赖遥控器 */
} ChassisControlSource;

/**
 * @brief 底盘运动学初始化配置
 * @param model 底盘几何参数和单轮速度上限
 * @param yaw_bias 平移偏航前馈补偿参数，在逆运动学中使用
 */
typedef struct {
    SteerWheelModel model;
    SteerWheelYawBiasConfig yaw_bias;
    ChassisControlSource control_source; /**< 初始化时确定；默认 REMOTE */
    uint32_t command_timeout_ms; /**< 外部指令有效期；0 默认 200 ms，最大 INT32_MAX */
} ChassisServiceConfig;

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
} ChassisFault;

/**
 * @brief 底盘服务只读状态快照
 */
typedef struct {
    float command_vx;      /**< 最近控制周期采用的 x 方向目标，m/s；不是实测值 */
    float command_vy;      /**< 最近控制周期采用的 y 方向目标，m/s；不是实测值 */
    float command_wz;      /**< 最近控制周期采用的 z 轴目标，rad/s；不是实测值 */
    uint32_t update_count; /**< 正常控制链完成次数；不含回零、故障和节拍外轮询 */
    ChassisFault fault;    /**< 当前锁存故障 */
    bool initialized;      /**< 服务是否初始化完成 */
    bool remote_online;    /**< 遥控链路是否在线 */
    bool remote_enabled;   /**< 遥控速度使能是否开启 */
    bool fault_latched;    /**< 是否存在锁存故障 */
    ChassisControlSource control_source; /**< 初始化时选择的控制来源 */
    bool ready;            /**< 初始化完成、回零完成且无锁存故障 */
    bool command_online;   /**< 当前来源的指令是否仍在有效期内 */
} ChassisServiceState;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 填充默认配置：原车型、关闭偏航补偿、遥控模式、200 ms 指令超时
 * @param out 非空配置输出；可在初始化前后调用，不改变运行实例
 * @return OK 或 INVALID_PARAM；不访问硬件
 */
ChassisServiceStatus chassis_service_get_default_config(ChassisServiceConfig* out);

/**
 * @brief 初始化底盘服务及全部依赖
 * @note 等价于获取默认配置后调用 init_with_config；不会执行 entry 中的 1000 ms 等待。
 * @return ChassisServiceStatus 状态码
 */
ChassisServiceStatus chassis_service_init(void);
/**
 * @brief 使用指定运动学配置初始化底盘服务及全部依赖
 * @param config 配置指针；初始化时复制参数，无需长期保留
 * @note 先完成宿主 HAL/外设初始化；函数保留原阻塞延时。成功后继续 update
 *       推进回零，get_state().ready 才是运行就绪标志。活动实例禁止重复初始化。
 * @retval INVALID_PARAM 空配置或非法控制来源/补偿/指令有效期
 * @retval INIT_FAILED 模型或设备依赖初始化失败，锁存初始化故障
 * @retval UNSAFE_STATE 已有活动设备实例；不会覆盖原实例
 * @return ChassisServiceStatus 状态码
 */
ChassisServiceStatus chassis_service_init_with_config(
    const ChassisServiceConfig* config);
/**
 * @brief 执行一次底盘服务轮询
 * @note 每轮维护接收，控制按内部 10 ms 节拍执行；OK 不表示发生了新控制周期。
 *       无新指令时也要继续调用，以检查超时、反馈、锁存和重试停止发送。
 * @return ChassisServiceStatus 状态码
 */
ChassisServiceStatus chassis_service_update(void);
/**
 * @brief 提交外部底盘速度，下一控制周期生效；仅外部模式可调用
 * @param vx x 方向 m/s，正方向沿用原运动学 x 轴
 * @param vy y 方向 m/s，正方向沿用原运动学 y 轴
 * @param wz z 轴 rad/s，正方向沿用原运动学旋转方向
 * @note 三轴必须有限且绝对值不超过 20；底层继续执行原限幅和方向修正。
 *       回零完成前仅接受全零；故障中仅接受全零用于显式清故障。
 *       以短于 command_timeout_ms 的间隔刷新；超时后按原斜坡归零。
 *       连续多次成功提交只保留最新一次；被拒绝的提交不刷新有效期。
 * @retval OK 指令已保存，尚不代表已发送或电机达到目标
 * @retval INVALID_PARAM 任一分量不是有限数或绝对值超过 20
 * @retval NOT_INITIALIZED 尚未完成初始化
 * @retval UNSAFE_STATE 遥控模式，或回零未完成时提交非零指令
 * @retval FAULT_LATCHED 锁存故障中提交非零指令；新零指令仍可用于恢复流程
 */
ChassisServiceStatus chassis_service_set_velocity(float vx, float vy, float wz);
/**
 * @brief 锁存人工停止故障并停止全部执行器
 * @note 已有锁存故障时保留首次故障原因。OK 表示锁存及停止发送已尝试，
 *       不保证发送成功或电机物理静止；应持续 update 以重试失败的停止发送。
 * @return ChassisServiceStatus 状态码
 */
ChassisServiceStatus chassis_service_stop(void);
/**
 * @brief 显式清故障：遥控模式需新安全帧；外部模式需故障后提交的新鲜全零指令
 * @note 成功后重新启动回零，外部旧指令作废；无故障时返回 UNSAFE_STATE。
 *       初始化未完成时返回 NOT_INITIALIZED；设备恢复发送失败返回 DEVICE_ERROR。
 * @return ChassisServiceStatus 状态码
 */
ChassisServiceStatus chassis_service_fault_clear(void);
/**
 * @brief 获取底盘服务状态快照
 * @param out 输出状态
 * @note 初始化前也可获取零状态；只有 out 为空时返回 INVALID_PARAM。
 *       快照只表示采样时刻的软件状态，不持有内部状态指针。
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
