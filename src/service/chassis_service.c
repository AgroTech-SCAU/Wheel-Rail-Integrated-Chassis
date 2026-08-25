/**
 * @file chassis_service.c
 * @brief 轮轨复合底盘服务实现
 */

#include "chassis_service.h"

#include "benmo_drive_motor.h"
#include "fs_ia10b.h"
#include "log.h"
#include "rs06_steer_motor.h"
#include "steer_wheel_kinematics.h"
#include "stm32_board_port.h"
#include "stm32_fdcan_port.h"
#include "stm32_time_port.h"
#include "stm32_uart_port.h"

#include <stddef.h>
#include <string.h>

// ! ========================= 宏 定 义 声 明 ========================= ! //

#define CHASSIS_LENGTH_M 0.725f
#define CHASSIS_WIDTH_M 0.730f
#define CHASSIS_WHEEL_RADIUS_M 0.0215f
#define CHASSIS_MAX_WHEEL_LINEAR_SPEED_M_S 0.45f
#define CHASSIS_UPDATE_PERIOD_MS 10u
#define CHASSIS_REMOTE_TIMEOUT_MS 200u
#define CHASSIS_DRIVE_FEEDBACK_TIMEOUT_MS 200u
#define CHASSIS_SEND_FAILURE_LIMIT 3u
#define CHASSIS_LOG_PERIOD_MS 500u
#define CHASSIS_RAD_S_TO_RPM 9.5492965855f

#define REMOTE_CH_RIGHT_X 0u
#define REMOTE_CH_RIGHT_Y 1u
#define REMOTE_CH_LEFT_X 3u
#define REMOTE_CH_SWB 5u
#define REMOTE_CH_VRB 9u
#define REMOTE_CENTER 1500u
#define REMOTE_SPAN 500.0f
#define REMOTE_DEADBAND 30u
#define REMOTE_SWITCH_LOW 2000u
#define REMOTE_SWITCH_HIGH 1000u
#define REMOTE_SWITCH_TOLERANCE 250u
#define REMOTE_ENABLE_THRESHOLD 1200u
#define REMOTE_FAST_LIMIT 20.0f
#define REMOTE_MEDIUM_LIMIT 10.0f
#define REMOTE_SLOW_LIMIT 5.0f
#define REMOTE_FINAL_LIMIT 20.0f

// ! ========================= 接 口 变 量 / Typedef 声 明
// ========================= ! //

/**
 * @brief 当前遥控挡位对应的三轴速度上限
 */
typedef struct {
    float max_vx;
    float max_vy;
    float max_wz;
} RemoteSpeedLimit;

/**
 * @brief 底盘服务内部运行上下文
 */
typedef struct {
    FsIa10b receiver;
    BenmoDriveMotor drive;
    Rs06SteerMotor steer;
    SteerWheel kinematics;
    ChassisServiceState state;
    volatile Stm32FdcanFrame pending_drive_frame;
    volatile bool drive_frame_pending;
    uint8_t drive_ids[BENMO_DRIVE_MOTOR_COUNT];
    uint8_t steer_ids[BENMO_DRIVE_MOTOR_COUNT];
    float last_steer_target[BENMO_DRIVE_MOTOR_COUNT];
    uint32_t last_update_ms;
    uint32_t last_log_ms;
    uint8_t consecutive_send_failures;
    uint32_t fault_clear_after_frame_count;
    bool stop_sent;
    bool fault_clear_armed;
} ChassisServiceContext;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static bool chassis_drive_send(uint32_t id, const uint8_t* data, uint8_t len);
static bool chassis_steer_send(uint32_t id, const uint8_t* data, uint8_t len);
static bool chassis_log_write(const char* data, uint32_t len);
static void chassis_uart_rx_complete(void* context);
static void chassis_uart_error(void* context);
static void chassis_drive_rx(const Stm32FdcanFrame* frame, void* context);
static float chassis_limit_float(float value, float minimum, float maximum);
static bool chassis_remote_switch_is(uint16_t value, uint16_t target);
static float chassis_remote_channel_to_norm(uint16_t value);
static float chassis_remote_channel_to_speed(uint16_t value, float maximum);
static RemoteSpeedLimit chassis_remote_speed_limit(uint16_t switch_value);
static void chassis_set_velocity(float vx, float vy, float wz);
static void chassis_update_remote_command(void);
static bool chassis_remote_is_safe_for_fault_clear(void);
static void chassis_send_stop_once(void);
static void chassis_latch_fault(ChassisFault fault);
static bool chassis_record_send_result(bool success, ChassisFault fault);
static bool chassis_initialize_steering(void);

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 底盘服务唯一运行实例
 */
static ChassisServiceContext s_chassis;

static const FsIa10bPortOps s_receiver_ops = {
    .start_receive = stm32_uart5_start_receive,
    .abort_receive = stm32_uart5_abort_receive,
    .receive_is_ready = stm32_uart5_receive_is_ready,
    .now_ms = stm32_time_now_ms,
    .critical_enter = stm32_critical_enter,
    .critical_exit = stm32_critical_exit,
};

static const BenmoDriveMotorPortOps s_drive_ops = {
    .send_standard = chassis_drive_send,
    .now_ms = stm32_time_now_ms,
    .critical_enter = stm32_critical_enter,
    .critical_exit = stm32_critical_exit,
};

static const Rs06SteerMotorPortOps s_steer_ops = {
    .send_extended = chassis_steer_send,
    .delay_ms = stm32_time_delay_ms,
};

static const LogPortOps s_log_ops = {
    .write = chassis_log_write,
};

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static bool chassis_drive_send(uint32_t id, const uint8_t* data, uint8_t len) {
    return stm32_fdcan_port_send_standard(STM32_FDCAN_BUS_DRIVE, id, data, len,
                                          0u);
}

static bool chassis_steer_send(uint32_t id, const uint8_t* data, uint8_t len) {
    return stm32_fdcan_port_send_extended(STM32_FDCAN_BUS_STEER, id, data, len,
                                          0u);
}

static bool chassis_log_write(const char* data, uint32_t len) {
    return stm32_usart1_write(data, len);
}

static void chassis_uart_rx_complete(void* context) {
    (void)fs_ia10b_on_rx_complete((FsIa10b*)context);
}

static void chassis_uart_error(void* context) {
    (void)fs_ia10b_on_rx_error((FsIa10b*)context);
}

static void chassis_drive_rx(const Stm32FdcanFrame* frame, void* context) {
    ChassisServiceContext* self = (ChassisServiceContext*)context;
    uint8_t i;
    if(frame == NULL || self == NULL ||
       frame->id_type != STM32_FDCAN_ID_STANDARD ||
       frame->len != BENMO_DRIVE_MOTOR_CAN_DATA_LEN) {
        return;
    }
    self->pending_drive_frame.id = frame->id;
    self->pending_drive_frame.id_type = frame->id_type;
    self->pending_drive_frame.len = frame->len;
    for(i = 0u; i < frame->len; ++i) {
        self->pending_drive_frame.data[i] = frame->data[i];
    }
    self->drive_frame_pending = true;
}

static float chassis_limit_float(float value, float minimum, float maximum) {
    if(value > maximum) {
        return maximum;
    }
    if(value < minimum) {
        return minimum;
    }
    return value;
}

static bool chassis_remote_switch_is(uint16_t value, uint16_t target) {
    uint16_t difference =
        value > target ? (uint16_t)(value - target) : (uint16_t)(target - value);
    return difference <= REMOTE_SWITCH_TOLERANCE;
}

static float chassis_remote_channel_to_norm(uint16_t value) {
    int32_t difference;
    if(value < 900u || value > 2100u) {
        return 0.0f;
    }
    difference = (int32_t)value - (int32_t)REMOTE_CENTER;
    if((difference < 0 && (uint32_t)-difference <= REMOTE_DEADBAND) ||
       (difference >= 0 && (uint32_t)difference <= REMOTE_DEADBAND)) {
        return 0.0f;
    }
    return chassis_limit_float((float)difference / REMOTE_SPAN, -1.0f, 1.0f);
}

static float chassis_remote_channel_to_speed(uint16_t value, float maximum) {
    float speed = chassis_remote_channel_to_norm(value) * maximum;
    speed = chassis_limit_float(speed, -maximum, maximum);
    return chassis_limit_float(speed, -REMOTE_FINAL_LIMIT, REMOTE_FINAL_LIMIT);
}

static RemoteSpeedLimit chassis_remote_speed_limit(uint16_t switch_value) {
    RemoteSpeedLimit limit;
    float selected = REMOTE_MEDIUM_LIMIT;
    if(chassis_remote_switch_is(switch_value, REMOTE_SWITCH_LOW)) {
        selected = REMOTE_FAST_LIMIT;
    }
    else if(chassis_remote_switch_is(switch_value, REMOTE_SWITCH_HIGH)) {
        selected = REMOTE_SLOW_LIMIT;
    }
    limit.max_vx = selected;
    limit.max_vy = selected;
    limit.max_wz = selected;
    return limit;
}

static void chassis_set_velocity(float vx, float vy, float wz) {
    s_chassis.state.command_vx = vx;
    s_chassis.state.command_vy = vy;
    s_chassis.state.command_wz = wz;
    s_chassis.kinematics.control.vx = vx;
    s_chassis.kinematics.control.vy = vy;
    s_chassis.kinematics.control.wz = wz;
}

static void chassis_update_remote_command(void) {
    FsIa10bData remote;
    RemoteSpeedLimit limit;
    if(fs_ia10b_get_data(&s_chassis.receiver, &remote) != FS_IA10B_STATUS_OK ||
       !fs_ia10b_is_online(&s_chassis.receiver, CHASSIS_REMOTE_TIMEOUT_MS)) {
        s_chassis.state.remote_online = false;
        s_chassis.state.remote_enabled = false;
        chassis_set_velocity(0.0f, 0.0f, 0.0f);
        return;
    }
    s_chassis.state.remote_online = true;
    if(remote.channel[REMOTE_CH_VRB] > REMOTE_ENABLE_THRESHOLD) {
        s_chassis.state.remote_enabled = false;
        chassis_set_velocity(0.0f, 0.0f, 0.0f);
        return;
    }
    s_chassis.state.remote_enabled = true;
    limit = chassis_remote_speed_limit(remote.channel[REMOTE_CH_SWB]);
    chassis_set_velocity(-chassis_remote_channel_to_speed(
                             remote.channel[REMOTE_CH_RIGHT_Y], limit.max_vx),
                         -chassis_remote_channel_to_speed(
                             remote.channel[REMOTE_CH_RIGHT_X], limit.max_vy),
                         -chassis_remote_channel_to_speed(
                             remote.channel[REMOTE_CH_LEFT_X], limit.max_wz));
}

static bool chassis_remote_is_safe_for_fault_clear(void) {
    FsIa10bData remote;
    (void)fs_ia10b_maintain(&s_chassis.receiver);
    if(fs_ia10b_get_data(&s_chassis.receiver, &remote) != FS_IA10B_STATUS_OK) {
        s_chassis.state.remote_online = false;
        s_chassis.state.remote_enabled = false;
        return false;
    }
    if(!s_chassis.fault_clear_armed) {
        s_chassis.fault_clear_after_frame_count = remote.frame_count;
        s_chassis.fault_clear_armed = true;
        return false;
    }
    if(remote.frame_count == s_chassis.fault_clear_after_frame_count) {
        return false;
    }
    s_chassis.fault_clear_after_frame_count = remote.frame_count;
    if(!fs_ia10b_is_online(&s_chassis.receiver, CHASSIS_REMOTE_TIMEOUT_MS)) {
        s_chassis.state.remote_online = false;
        s_chassis.state.remote_enabled = false;
        return false;
    }
    s_chassis.state.remote_online = true;
    s_chassis.state.remote_enabled =
        remote.channel[REMOTE_CH_VRB] <= REMOTE_ENABLE_THRESHOLD;
    return !s_chassis.state.remote_enabled;
}

static void chassis_send_stop_once(void) {
    uint8_t i;
    bool success = true;
    if(s_chassis.stop_sent) {
        return;
    }
    if(s_chassis.drive.initialized &&
       benmo_drive_motor_stop_all(&s_chassis.drive) != BENMO_DRIVE_STATUS_OK) {
        success = false;
    }
    if(rs06_steer_motor_is_initialized(&s_chassis.steer)) {
        for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
            if(rs06_steer_motor_stop(&s_chassis.steer, s_chassis.steer_ids[i]) !=
               RS06_STEER_STATUS_OK) {
                success = false;
            }
        }
    }
    s_chassis.stop_sent = success;
}

static void chassis_latch_fault(ChassisFault fault) {
    bool first_fault = !s_chassis.state.fault_latched;
    if(first_fault) {
        s_chassis.state.fault = fault;
        s_chassis.state.fault_latched = true;
    }
    s_chassis.fault_clear_armed = false;
    chassis_set_velocity(0.0f, 0.0f, 0.0f);
    chassis_send_stop_once();
    if(first_fault) {
        (void)log_error("fault latched: %s", chassis_service_fault_str(fault));
    }
}

static bool chassis_record_send_result(bool success, ChassisFault fault) {
    if(success) {
        s_chassis.consecutive_send_failures = 0u;
        return true;
    }
    if(s_chassis.consecutive_send_failures < 0xFFu) {
        s_chassis.consecutive_send_failures++;
    }
    if(s_chassis.consecutive_send_failures >= CHASSIS_SEND_FAILURE_LIMIT) {
        chassis_latch_fault(fault);
    }
    return false;
}

static bool chassis_initialize_steering(void) {
    uint8_t i;
    stm32_time_delay_ms(100u);
    for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        if(rs06_steer_motor_set_mode(&s_chassis.steer, s_chassis.steer_ids[i],
                                     RS06_STEER_MODE_PP) != RS06_STEER_STATUS_OK) {
            return false;
        }
        stm32_time_delay_ms(5u);
        if(rs06_steer_motor_enable(&s_chassis.steer, s_chassis.steer_ids[i]) !=
           RS06_STEER_STATUS_OK) {
            return false;
        }
        stm32_time_delay_ms(5u);
        if(rs06_steer_motor_set_position_target(&s_chassis.steer,
                                                s_chassis.steer_ids[i],
                                                0.0f) != RS06_STEER_STATUS_OK) {
            return false;
        }
        stm32_time_delay_ms(5u);
    }
    return true;
}

static void chassis_log_initialized(void) {
    (void)log_info("system initialized");
}

// ! ========================= 接 口 函 数 实 现 ========================= ! //

ChassisServiceStatus chassis_service_init(void) {
    FsIa10bConfig receiver_config;
    BenmoDriveMotorConfig drive_config;
    Rs06SteerMotorConfig steer_config;
    SteerWheelModel model;
    LogConfig log_config;
    uint8_t i;

    memset(&s_chassis, 0, sizeof(s_chassis));
    for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        s_chassis.drive_ids[i] = (uint8_t)(i + 1u);
        s_chassis.steer_ids[i] = (uint8_t)(i + 5u);
    }
    model.length = CHASSIS_LENGTH_M;
    model.width = CHASSIS_WIDTH_M;
    model.wheel_radius = CHASSIS_WHEEL_RADIUS_M;
    model.max_wheel_linear_speed = CHASSIS_MAX_WHEEL_LINEAR_SPEED_M_S;
    if(steer_wheel_init(&s_chassis.kinematics, model) != STEER_WHEEL_OK) {
        chassis_latch_fault(CHASSIS_FAULT_INITIALIZATION);
        return CHASSIS_SERVICE_STATUS_INIT_FAILED;
    }

    stm32_board_chassis_power_enable();
    if(!stm32_fdcan_port_register_rx_callback(STM32_FDCAN_BUS_DRIVE,
                                              chassis_drive_rx, &s_chassis) ||
       !stm32_fdcan_port_init(STM32_FDCAN_BUS_DRIVE)) {
        chassis_latch_fault(CHASSIS_FAULT_INITIALIZATION);
        return CHASSIS_SERVICE_STATUS_INIT_FAILED;
    }
    drive_config.ops = &s_drive_ops;
    drive_config.feedback_timeout_ms = CHASSIS_DRIVE_FEEDBACK_TIMEOUT_MS;
    if(benmo_drive_motor_init(&s_chassis.drive, &drive_config) !=
       BENMO_DRIVE_STATUS_OK) {
        chassis_latch_fault(CHASSIS_FAULT_INITIALIZATION);
        return CHASSIS_SERVICE_STATUS_INIT_FAILED;
    }
    if(benmo_drive_motor_stop_all(&s_chassis.drive) != BENMO_DRIVE_STATUS_OK) {
        chassis_latch_fault(CHASSIS_FAULT_INITIALIZATION);
        return CHASSIS_SERVICE_STATUS_INIT_FAILED;
    }

    if(!stm32_fdcan_port_init(STM32_FDCAN_BUS_STEER)) {
        chassis_latch_fault(CHASSIS_FAULT_INITIALIZATION);
        return CHASSIS_SERVICE_STATUS_INIT_FAILED;
    }
    steer_config.ops = &s_steer_ops;
    steer_config.host_id = RS06_STEER_HOST_ID;
    if(rs06_steer_motor_init(&s_chassis.steer, &steer_config) !=
           RS06_STEER_STATUS_OK ||
       !chassis_initialize_steering()) {
        chassis_latch_fault(CHASSIS_FAULT_INITIALIZATION);
        return CHASSIS_SERVICE_STATUS_INIT_FAILED;
    }

    if(!stm32_uart5_register_callbacks(
           chassis_uart_rx_complete, chassis_uart_error, &s_chassis.receiver)) {
        chassis_latch_fault(CHASSIS_FAULT_INITIALIZATION);
        return CHASSIS_SERVICE_STATUS_INIT_FAILED;
    }
    receiver_config.ops = &s_receiver_ops;
    if(fs_ia10b_init(&s_chassis.receiver, &receiver_config) !=
       FS_IA10B_STATUS_OK) {
        chassis_latch_fault(CHASSIS_FAULT_INITIALIZATION);
        return CHASSIS_SERVICE_STATUS_INIT_FAILED;
    }

    log_config.ops = &s_log_ops;
    log_config.level = LOG_LEVEL_INFO;
    log_config.enable_color = false;
    log_config.async_write = false;
    if(log_init(&log_config) != LOG_STATUS_OK) {
        chassis_latch_fault(CHASSIS_FAULT_INITIALIZATION);
        return CHASSIS_SERVICE_STATUS_INIT_FAILED;
    }

    s_chassis.state.initialized = true;
    chassis_log_initialized();
    s_chassis.last_update_ms = stm32_time_now_ms();
    s_chassis.last_log_ms = s_chassis.last_update_ms;
    (void)benmo_drive_motor_reset_feedback_monitor(&s_chassis.drive);
    return CHASSIS_SERVICE_STATUS_OK;
}

ChassisServiceStatus chassis_service_update(void) {
    uint32_t now;
    bool drive_sends_ok = true;
    bool steer_sends_ok = true;
    uint8_t i;
    if(!s_chassis.state.initialized) {
        if(s_chassis.state.fault_latched) {
            chassis_send_stop_once();
            return CHASSIS_SERVICE_STATUS_FAULT_LATCHED;
        }
        return CHASSIS_SERVICE_STATUS_NOT_INITIALIZED;
    }
    (void)fs_ia10b_maintain(&s_chassis.receiver);
    if(s_chassis.drive_frame_pending) {
        Stm32FdcanFrame frame;
        uint8_t i;
        uint32_t critical_state = stm32_critical_enter();
        frame.id = s_chassis.pending_drive_frame.id;
        frame.id_type = s_chassis.pending_drive_frame.id_type;
        frame.len = s_chassis.pending_drive_frame.len;
        for(i = 0u; i < frame.len; ++i) {
            frame.data[i] = s_chassis.pending_drive_frame.data[i];
        }
        s_chassis.drive_frame_pending = false;
        stm32_critical_exit(critical_state);
        (void)benmo_drive_motor_handle_feedback(&s_chassis.drive, frame.id,
                                                frame.data, frame.len);
    }
    if(s_chassis.state.fault_latched) {
        chassis_send_stop_once();
        return CHASSIS_SERVICE_STATUS_FAULT_LATCHED;
    }

    now = stm32_time_now_ms();
    if((uint32_t)(now - s_chassis.last_update_ms) < CHASSIS_UPDATE_PERIOD_MS) {
        return CHASSIS_SERVICE_STATUS_OK;
    }
    s_chassis.last_update_ms = now;
    chassis_update_remote_command();
    if(steer_wheel_ik(&s_chassis.kinematics) != STEER_WHEEL_OK) {
        chassis_latch_fault(CHASSIS_FAULT_KINEMATICS);
        return CHASSIS_SERVICE_STATUS_KINEMATICS_ERROR;
    }
    if(steer_wheel_apply_legacy_57_correction(&s_chassis.kinematics) !=
       STEER_WHEEL_OK) {
        chassis_latch_fault(CHASSIS_FAULT_KINEMATICS);
        return CHASSIS_SERVICE_STATUS_KINEMATICS_ERROR;
    }
    if(steer_wheel_optimize_targets(&s_chassis.kinematics,
                                    s_chassis.last_steer_target) !=
       STEER_WHEEL_OK) {
        chassis_latch_fault(CHASSIS_FAULT_KINEMATICS);
        return CHASSIS_SERVICE_STATUS_KINEMATICS_ERROR;
    }

    for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        int16_t rpm = (int16_t)(s_chassis.kinematics.control.wheels[i].wheel_omega *
                                CHASSIS_RAD_S_TO_RPM);
        bool steer_send_ok;
        (void)benmo_drive_motor_set_target_rpm(&s_chassis.drive,
                                               s_chassis.drive_ids[i], rpm);
        steer_send_ok =
            rs06_steer_motor_set_mode(&s_chassis.steer,
                                      s_chassis.steer_ids[i],
                                      RS06_STEER_MODE_PP) ==
                RS06_STEER_STATUS_OK &&
            rs06_steer_motor_enable(&s_chassis.steer,
                                    s_chassis.steer_ids[i]) ==
                RS06_STEER_STATUS_OK &&
            rs06_steer_motor_set_position_target(
                &s_chassis.steer, s_chassis.steer_ids[i],
                s_chassis.kinematics.control.wheels[i].steer_angle) ==
                RS06_STEER_STATUS_OK;
        if(!steer_send_ok) {
            steer_sends_ok = false;
        }
        else {
            s_chassis.last_steer_target[i] =
                s_chassis.kinematics.control.wheels[i].steer_angle;
        }
    }
    if(benmo_drive_motor_update(&s_chassis.drive) != BENMO_DRIVE_STATUS_OK ||
       benmo_drive_motor_request_next_feedback(&s_chassis.drive) !=
           BENMO_DRIVE_STATUS_OK) {
        drive_sends_ok = false;
    }
    if(!steer_sends_ok) {
        (void)chassis_record_send_result(false, CHASSIS_FAULT_STEER_TRANSMIT);
    }
    else if(!drive_sends_ok) {
        (void)chassis_record_send_result(false, CHASSIS_FAULT_DRIVE_TRANSMIT);
    }
    else {
        (void)chassis_record_send_result(true, CHASSIS_FAULT_NONE);
    }
    if(s_chassis.state.fault_latched) {
        return CHASSIS_SERVICE_STATUS_FAULT_LATCHED;
    }
    if(benmo_drive_motor_check_feedback(&s_chassis.drive) ==
       BENMO_DRIVE_STATUS_FEEDBACK_TIMEOUT) {
        chassis_latch_fault(CHASSIS_FAULT_DRIVE_FEEDBACK_TIMEOUT);
        return CHASSIS_SERVICE_STATUS_FAULT_LATCHED;
    }

    s_chassis.state.update_count++;
    if((uint32_t)(now - s_chassis.last_log_ms) >= CHASSIS_LOG_PERIOD_MS) {
        s_chassis.last_log_ms = now;
        log_info("[遥控指令 x1000] vx=%ld vy=%ld wz=%ld",
                 (long)(s_chassis.state.command_vx * 1000.0f),
                 (long)(s_chassis.state.command_vy * 1000.0f),
                 (long)(s_chassis.state.command_wz * 1000.0f));
    }
    return (drive_sends_ok && steer_sends_ok)
               ? CHASSIS_SERVICE_STATUS_OK
               : CHASSIS_SERVICE_STATUS_DEVICE_ERROR;
}

ChassisServiceStatus chassis_service_stop(void) {
    if(!s_chassis.drive.initialized && !s_chassis.steer.initialized) {
        return CHASSIS_SERVICE_STATUS_NOT_INITIALIZED;
    }
    s_chassis.stop_sent = false;
    s_chassis.fault_clear_armed = false;
    chassis_latch_fault(CHASSIS_FAULT_MANUAL_STOP);
    return CHASSIS_SERVICE_STATUS_OK;
}

ChassisServiceStatus chassis_service_fault_clear(void) {
    uint8_t i;
    if(!s_chassis.state.initialized) {
        return CHASSIS_SERVICE_STATUS_NOT_INITIALIZED;
    }
    if(!s_chassis.state.fault_latched ||
       !chassis_remote_is_safe_for_fault_clear()) {
        return CHASSIS_SERVICE_STATUS_UNSAFE_STATE;
    }
    for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        if(rs06_steer_motor_set_mode(&s_chassis.steer, s_chassis.steer_ids[i],
                                     RS06_STEER_MODE_PP) != RS06_STEER_STATUS_OK ||
           rs06_steer_motor_enable(&s_chassis.steer, s_chassis.steer_ids[i]) !=
               RS06_STEER_STATUS_OK ||
           rs06_steer_motor_set_position_target(&s_chassis.steer,
                                                s_chassis.steer_ids[i],
                                                0.0f) != RS06_STEER_STATUS_OK) {
            return CHASSIS_SERVICE_STATUS_DEVICE_ERROR;
        }
    }
    (void)benmo_drive_motor_reset_feedback_monitor(&s_chassis.drive);
    s_chassis.consecutive_send_failures = 0u;
    s_chassis.stop_sent = false;
    s_chassis.fault_clear_armed = false;
    s_chassis.state.fault = CHASSIS_FAULT_NONE;
    s_chassis.state.fault_latched = false;
    chassis_set_velocity(0.0f, 0.0f, 0.0f);
    (void)log_info("[安全] fault cleared by explicit request");
    return CHASSIS_SERVICE_STATUS_OK;
}

ChassisServiceStatus chassis_service_get_state(ChassisServiceState* out) {
    uint32_t critical_state;
    if(out == NULL) {
        return CHASSIS_SERVICE_STATUS_INVALID_PARAM;
    }
    critical_state = stm32_critical_enter();
    *out = s_chassis.state;
    stm32_critical_exit(critical_state);
    return CHASSIS_SERVICE_STATUS_OK;
}

const char* chassis_service_status_str(ChassisServiceStatus status) {
    switch(status) {
        case CHASSIS_SERVICE_STATUS_OK:
            return "OK";
        case CHASSIS_SERVICE_STATUS_INVALID_PARAM:
            return "INVALID_PARAM";
        case CHASSIS_SERVICE_STATUS_NOT_INITIALIZED:
            return "NOT_INITIALIZED";
        case CHASSIS_SERVICE_STATUS_INIT_FAILED:
            return "INIT_FAILED";
        case CHASSIS_SERVICE_STATUS_DEVICE_ERROR:
            return "DEVICE_ERROR";
        case CHASSIS_SERVICE_STATUS_KINEMATICS_ERROR:
            return "KINEMATICS_ERROR";
        case CHASSIS_SERVICE_STATUS_FAULT_LATCHED:
            return "FAULT_LATCHED";
        case CHASSIS_SERVICE_STATUS_UNSAFE_STATE:
            return "UNSAFE_STATE";
        default:
            return "UNKNOWN";
    }
}

const char* chassis_service_fault_str(ChassisFault fault) {
    switch(fault) {
        case CHASSIS_FAULT_NONE:
            return "NONE";
        case CHASSIS_FAULT_INITIALIZATION:
            return "INITIALIZATION";
        case CHASSIS_FAULT_DRIVE_TRANSMIT:
            return "DRIVE_TRANSMIT";
        case CHASSIS_FAULT_STEER_TRANSMIT:
            return "STEER_TRANSMIT";
        case CHASSIS_FAULT_DRIVE_FEEDBACK_TIMEOUT:
            return "DRIVE_FEEDBACK_TIMEOUT";
        case CHASSIS_FAULT_KINEMATICS:
            return "KINEMATICS";
        case CHASSIS_FAULT_MANUAL_STOP:
            return "MANUAL_STOP";
        default:
            return "UNKNOWN";
    }
}
