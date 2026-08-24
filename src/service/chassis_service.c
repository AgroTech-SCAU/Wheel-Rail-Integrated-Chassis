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

#define CHASSIS_LENGTH_M 0.725F
#define CHASSIS_WIDTH_M 0.730F
#define CHASSIS_WHEEL_RADIUS_M 0.0215F
#define CHASSIS_MAX_WHEEL_LINEAR_SPEED_M_S 0.45F
#define CHASSIS_UPDATE_PERIOD_MS 10U
#define CHASSIS_REMOTE_TIMEOUT_MS 200U
#define CHASSIS_DRIVE_FEEDBACK_TIMEOUT_MS 200U
#define CHASSIS_SEND_FAILURE_LIMIT 3U
#define CHASSIS_LOG_PERIOD_MS 500U
#define CHASSIS_RAD_S_TO_RPM 9.5492965855F
#define CHASSIS_DRIVE_RX_QUEUE_DEPTH 8U

#define REMOTE_CH_RIGHT_X 0U
#define REMOTE_CH_RIGHT_Y 1U
#define REMOTE_CH_LEFT_X 3U
#define REMOTE_CH_SWB 5U
#define REMOTE_CH_VRB 9U
#define REMOTE_CENTER 1500U
#define REMOTE_SPAN 500.0F
#define REMOTE_DEADBAND 30U
#define REMOTE_SWITCH_LOW 2000U
#define REMOTE_SWITCH_HIGH 1000U
#define REMOTE_SWITCH_TOLERANCE 250U
#define REMOTE_ENABLE_THRESHOLD 1200U
#define REMOTE_FAST_LIMIT 20.0F
#define REMOTE_MEDIUM_LIMIT 10.0F
#define REMOTE_SLOW_LIMIT 5.0F
#define REMOTE_FINAL_LIMIT 20.0F

typedef struct {
    float max_vx;
    float max_vy;
    float max_wz;
} RemoteSpeedLimit;

typedef struct {
    FsIa10b receiver;
    BenmoDriveMotor drive;
    Rs06SteerMotor steer;
    SteerWheel kinematics;
    ChassisServiceState state;
    volatile Stm32FdcanFrame drive_rx_queue[CHASSIS_DRIVE_RX_QUEUE_DEPTH];
    volatile uint8_t drive_rx_head;
    volatile uint8_t drive_rx_tail;
    volatile bool drive_rx_overflow;
    uint8_t drive_ids[BENMO_DRIVE_MOTOR_COUNT];
    uint8_t steer_ids[BENMO_DRIVE_MOTOR_COUNT];
    uint32_t last_update_ms;
    uint32_t last_log_ms;
    uint8_t consecutive_send_failures;
    uint32_t fault_clear_after_frame_count;
    bool stop_sent;
    bool fault_clear_armed;
} ChassisServiceContext;

static ChassisServiceContext g_chassis;

static bool chassis_drive_send(uint32_t id, const uint8_t* data, uint8_t len)
{
    return stm32_fdcan_port_send_standard(STM32_FDCAN_BUS_DRIVE, id, data, len, 0U);
}

static bool chassis_steer_send(uint32_t id, const uint8_t* data, uint8_t len)
{
    return stm32_fdcan_port_send_extended(STM32_FDCAN_BUS_STEER, id, data, len, 0U);
}

static bool chassis_log_write(const char* data, uint32_t len)
{
    return stm32_usart1_write(data, len);
}

static const FsIa10bPortOps g_receiver_ops = {
    .start_receive = stm32_uart5_start_receive,
    .abort_receive = stm32_uart5_abort_receive,
    .receive_is_ready = stm32_uart5_receive_is_ready,
    .now_ms = stm32_time_now_ms,
    .critical_enter = stm32_critical_enter,
    .critical_exit = stm32_critical_exit,
};

static const BenmoDriveMotorPortOps g_drive_ops = {
    .send_standard = chassis_drive_send,
    .now_ms = stm32_time_now_ms,
    .critical_enter = stm32_critical_enter,
    .critical_exit = stm32_critical_exit,
};

static const Rs06SteerMotorPortOps g_steer_ops = {
    .send_extended = chassis_steer_send,
    .delay_ms = stm32_time_delay_ms,
};

static const LogPortOps g_log_ops = {
    .write = chassis_log_write,
};

static void chassis_uart_rx_complete(void* context)
{
    (void)fs_ia10b_on_rx_complete((FsIa10b*)context);
}

static void chassis_uart_error(void* context)
{
    (void)fs_ia10b_on_rx_error((FsIa10b*)context);
}

static void chassis_drive_rx(const Stm32FdcanFrame* frame, void* context)
{
    ChassisServiceContext* self = (ChassisServiceContext*)context;
    uint8_t next_head;
    uint8_t i;
    if (frame == NULL || self == NULL ||
        frame->id_type != STM32_FDCAN_ID_STANDARD ||
        frame->len != BENMO_DRIVE_MOTOR_CAN_DATA_LEN) {
        return;
    }
    next_head = (uint8_t)((self->drive_rx_head + 1U) % CHASSIS_DRIVE_RX_QUEUE_DEPTH);
    if (next_head == self->drive_rx_tail) {
        self->drive_rx_overflow = true;
        return;
    }
    self->drive_rx_queue[self->drive_rx_head].id = frame->id;
    self->drive_rx_queue[self->drive_rx_head].id_type = frame->id_type;
    self->drive_rx_queue[self->drive_rx_head].len = frame->len;
    for (i = 0U; i < frame->len; ++i) {
        self->drive_rx_queue[self->drive_rx_head].data[i] = frame->data[i];
    }
    self->drive_rx_head = next_head;
}

static float chassis_limit_float(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

static bool chassis_remote_switch_is(uint16_t value, uint16_t target)
{
    uint16_t difference = value > target ? (uint16_t)(value - target)
                                         : (uint16_t)(target - value);
    return difference <= REMOTE_SWITCH_TOLERANCE;
}

static float chassis_remote_channel_to_norm(uint16_t value)
{
    int32_t difference;
    if (value < 900U || value > 2100U) {
        return 0.0F;
    }
    difference = (int32_t)value - (int32_t)REMOTE_CENTER;
    if ((difference < 0 && (uint32_t)-difference <= REMOTE_DEADBAND) ||
        (difference >= 0 && (uint32_t)difference <= REMOTE_DEADBAND)) {
        return 0.0F;
    }
    return chassis_limit_float((float)difference / REMOTE_SPAN, -1.0F, 1.0F);
}

static float chassis_remote_channel_to_speed(uint16_t value, float maximum)
{
    float speed = chassis_remote_channel_to_norm(value) * maximum;
    speed = chassis_limit_float(speed, -maximum, maximum);
    return chassis_limit_float(speed, -REMOTE_FINAL_LIMIT, REMOTE_FINAL_LIMIT);
}

static RemoteSpeedLimit chassis_remote_speed_limit(uint16_t switch_value)
{
    RemoteSpeedLimit limit;
    float selected = REMOTE_MEDIUM_LIMIT;
    if (chassis_remote_switch_is(switch_value, REMOTE_SWITCH_LOW)) {
        selected = REMOTE_FAST_LIMIT;
    } else if (chassis_remote_switch_is(switch_value, REMOTE_SWITCH_HIGH)) {
        selected = REMOTE_SLOW_LIMIT;
    }
    limit.max_vx = selected;
    limit.max_vy = selected;
    limit.max_wz = selected;
    return limit;
}

static void chassis_set_velocity(float vx, float vy, float wz)
{
    g_chassis.state.command_vx = vx;
    g_chassis.state.command_vy = vy;
    g_chassis.state.command_wz = wz;
    g_chassis.kinematics.control.vx = vx;
    g_chassis.kinematics.control.vy = vy;
    g_chassis.kinematics.control.wz = wz;
}

static void chassis_update_remote_command(void)
{
    FsIa10bData remote;
    RemoteSpeedLimit limit;
    if (fs_ia10b_get_data(&g_chassis.receiver, &remote) != FS_IA10B_STATUS_OK ||
        !fs_ia10b_is_online(&g_chassis.receiver, CHASSIS_REMOTE_TIMEOUT_MS)) {
        g_chassis.state.remote_online = false;
        g_chassis.state.remote_enabled = false;
        chassis_set_velocity(0.0F, 0.0F, 0.0F);
        return;
    }
    g_chassis.state.remote_online = true;
    if (remote.channel[REMOTE_CH_VRB] > REMOTE_ENABLE_THRESHOLD) {
        g_chassis.state.remote_enabled = false;
        chassis_set_velocity(0.0F, 0.0F, 0.0F);
        return;
    }
    g_chassis.state.remote_enabled = true;
    limit = chassis_remote_speed_limit(remote.channel[REMOTE_CH_SWB]);
    chassis_set_velocity(
        -chassis_remote_channel_to_speed(remote.channel[REMOTE_CH_RIGHT_Y], limit.max_vx),
        -chassis_remote_channel_to_speed(remote.channel[REMOTE_CH_RIGHT_X], limit.max_vy),
        -chassis_remote_channel_to_speed(remote.channel[REMOTE_CH_LEFT_X], limit.max_wz));
}

static bool chassis_remote_is_safe_for_fault_clear(void)
{
    FsIa10bData remote;
    (void)fs_ia10b_maintain(&g_chassis.receiver);
    if (fs_ia10b_get_data(&g_chassis.receiver, &remote) != FS_IA10B_STATUS_OK) {
        g_chassis.state.remote_online = false;
        g_chassis.state.remote_enabled = false;
        return false;
    }
    if (!g_chassis.fault_clear_armed) {
        g_chassis.fault_clear_after_frame_count = remote.frame_count;
        g_chassis.fault_clear_armed = true;
        return false;
    }
    if (remote.frame_count == g_chassis.fault_clear_after_frame_count) {
        return false;
    }
    g_chassis.fault_clear_after_frame_count = remote.frame_count;
    if (!fs_ia10b_is_online(&g_chassis.receiver, CHASSIS_REMOTE_TIMEOUT_MS)) {
        g_chassis.state.remote_online = false;
        g_chassis.state.remote_enabled = false;
        return false;
    }
    g_chassis.state.remote_online = true;
    g_chassis.state.remote_enabled =
        remote.channel[REMOTE_CH_VRB] <= REMOTE_ENABLE_THRESHOLD;
    return !g_chassis.state.remote_enabled;
}

static void chassis_send_stop_once(void)
{
    uint8_t i;
    bool success = true;
    if (g_chassis.stop_sent) {
        return;
    }
    if (g_chassis.drive.initialized &&
        benmo_drive_motor_stop_all(&g_chassis.drive) != BENMO_DRIVE_STATUS_OK) {
        success = false;
    }
    if (rs06_steer_motor_is_initialized(&g_chassis.steer)) {
        for (i = 0U; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
            if (rs06_steer_motor_stop(&g_chassis.steer, g_chassis.steer_ids[i]) !=
                RS06_STEER_STATUS_OK) {
                success = false;
            }
        }
    }
    g_chassis.stop_sent = success;
}

static void chassis_latch_fault(ChassisFault fault)
{
    g_chassis.state.fault = fault;
    g_chassis.state.fault_latched = true;
    g_chassis.fault_clear_armed = false;
    chassis_set_velocity(0.0F, 0.0F, 0.0F);
    chassis_send_stop_once();
    (void)log_error("[安全] fault latched: %s", chassis_service_fault_str(fault));
}

static bool chassis_record_send_result(bool success, ChassisFault fault)
{
    if (success) {
        g_chassis.consecutive_send_failures = 0U;
        return true;
    }
    if (g_chassis.consecutive_send_failures < 0xFFU) {
        g_chassis.consecutive_send_failures++;
    }
    if (g_chassis.consecutive_send_failures >= CHASSIS_SEND_FAILURE_LIMIT) {
        chassis_latch_fault(fault);
    }
    return false;
}

static bool chassis_initialize_steering(void)
{
    uint8_t i;
    stm32_time_delay_ms(100U);
    for (i = 0U; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        if (rs06_steer_motor_set_mode(&g_chassis.steer, g_chassis.steer_ids[i],
                                      RS06_STEER_MODE_PP) != RS06_STEER_STATUS_OK) {
            return false;
        }
        stm32_time_delay_ms(5U);
        if (rs06_steer_motor_enable(&g_chassis.steer, g_chassis.steer_ids[i]) !=
            RS06_STEER_STATUS_OK) {
            return false;
        }
        stm32_time_delay_ms(5U);
        if (rs06_steer_motor_set_position_target(&g_chassis.steer,
                                                 g_chassis.steer_ids[i], 0.0F) !=
            RS06_STEER_STATUS_OK) {
            return false;
        }
        stm32_time_delay_ms(5U);
    }
    return true;
}

static void chassis_log_configuration(void)
{
    log_info("============================================");
    log_info("  轮轨复合底盘 - 分层架构初始化报告");
    log_info("============================================");
    log_info("[系统] STM32H723VGT6, app -> service -> device/domain/infra -> platform");
    log_info("[CAN] FDCAN1 驱动电机 500 kbit/s, FDCAN2 RS06 1 Mbit/s");
    log_info("[驱动] ID=1..4, 限幅=%d..%d RPM, 斜坡=%d RPM/10ms",
             BENMO_DRIVE_MOTOR_RPM_MIN, BENMO_DRIVE_MOTOR_RPM_MAX,
             BENMO_DRIVE_MOTOR_RAMP_STEP_RPM);
    log_info("[舵向] RS06 ID=5..8, PP 模式, ID 5/7 保留旧方向修正");
    log_info("[底盘] length=%.3f m width=%.3f m radius=%.4f m max=%.2f m/s",
             (double)CHASSIS_LENGTH_M, (double)CHASSIS_WIDTH_M,
             (double)CHASSIS_WHEEL_RADIUS_M,
             (double)CHASSIS_MAX_WHEEL_LINEAR_SPEED_M_S);
    log_info("[遥控] UART5 iBUS, 失联/VRB关闭时保持使能并下发零速");
    log_info("[安全] 初始化/连续发送/反馈超时故障锁存并停止全部执行器");
    log_info("============================================");
}

ChassisServiceStatus chassis_service_init(void)
{
    FsIa10bConfig receiver_config;
    BenmoDriveMotorConfig drive_config;
    Rs06SteerMotorConfig steer_config;
    SteerWheelModel model;
    LogConfig log_config;
    uint8_t i;

    memset(&g_chassis, 0, sizeof(g_chassis));
    for (i = 0U; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        g_chassis.drive_ids[i] = (uint8_t)(i + 1U);
        g_chassis.steer_ids[i] = (uint8_t)(i + 5U);
    }
    model.length = CHASSIS_LENGTH_M;
    model.width = CHASSIS_WIDTH_M;
    model.wheel_radius = CHASSIS_WHEEL_RADIUS_M;
    model.max_wheel_linear_speed = CHASSIS_MAX_WHEEL_LINEAR_SPEED_M_S;
    if (steer_wheel_init(&g_chassis.kinematics, model) != STEER_WHEEL_OK) {
        chassis_latch_fault(CHASSIS_FAULT_INITIALIZATION);
        return CHASSIS_SERVICE_STATUS_INIT_FAILED;
    }

    stm32_board_chassis_power_enable();
    if (!stm32_fdcan_port_register_rx_callback(STM32_FDCAN_BUS_DRIVE,
                                                chassis_drive_rx, &g_chassis) ||
        !stm32_fdcan_port_init(STM32_FDCAN_BUS_DRIVE)) {
        chassis_latch_fault(CHASSIS_FAULT_INITIALIZATION);
        return CHASSIS_SERVICE_STATUS_INIT_FAILED;
    }
    drive_config.ops = &g_drive_ops;
    drive_config.feedback_timeout_ms = CHASSIS_DRIVE_FEEDBACK_TIMEOUT_MS;
    if (benmo_drive_motor_init(&g_chassis.drive, &drive_config) !=
        BENMO_DRIVE_STATUS_OK) {
        chassis_latch_fault(CHASSIS_FAULT_INITIALIZATION);
        return CHASSIS_SERVICE_STATUS_INIT_FAILED;
    }
    if (benmo_drive_motor_stop_all(&g_chassis.drive) != BENMO_DRIVE_STATUS_OK) {
        chassis_latch_fault(CHASSIS_FAULT_INITIALIZATION);
        return CHASSIS_SERVICE_STATUS_INIT_FAILED;
    }

    if (!stm32_fdcan_port_init(STM32_FDCAN_BUS_STEER)) {
        chassis_latch_fault(CHASSIS_FAULT_INITIALIZATION);
        return CHASSIS_SERVICE_STATUS_INIT_FAILED;
    }
    steer_config.ops = &g_steer_ops;
    steer_config.host_id = RS06_STEER_HOST_ID;
    if (rs06_steer_motor_init(&g_chassis.steer, &steer_config) !=
            RS06_STEER_STATUS_OK ||
        !chassis_initialize_steering()) {
        chassis_latch_fault(CHASSIS_FAULT_INITIALIZATION);
        return CHASSIS_SERVICE_STATUS_INIT_FAILED;
    }

    if (!stm32_uart5_register_callbacks(chassis_uart_rx_complete,
                                         chassis_uart_error,
                                         &g_chassis.receiver)) {
        chassis_latch_fault(CHASSIS_FAULT_INITIALIZATION);
        return CHASSIS_SERVICE_STATUS_INIT_FAILED;
    }
    receiver_config.ops = &g_receiver_ops;
    if (fs_ia10b_init(&g_chassis.receiver, &receiver_config) != FS_IA10B_STATUS_OK) {
        chassis_latch_fault(CHASSIS_FAULT_INITIALIZATION);
        return CHASSIS_SERVICE_STATUS_INIT_FAILED;
    }

    log_config.ops = &g_log_ops;
    log_config.level = LOG_LEVEL_INFO;
    log_config.enable_color = true;
    log_config.async_write = false;
    if (log_init(&log_config) != LOG_STATUS_OK) {
        chassis_latch_fault(CHASSIS_FAULT_INITIALIZATION);
        return CHASSIS_SERVICE_STATUS_INIT_FAILED;
    }

    g_chassis.state.initialized = true;
    chassis_log_configuration();
    g_chassis.last_update_ms = stm32_time_now_ms();
    g_chassis.last_log_ms = g_chassis.last_update_ms;
    (void)benmo_drive_motor_reset_feedback_monitor(&g_chassis.drive);
    return CHASSIS_SERVICE_STATUS_OK;
}

ChassisServiceStatus chassis_service_update(void)
{
    uint32_t now;
    bool drive_sends_ok = true;
    bool steer_sends_ok = true;
    uint8_t i;
    if (!g_chassis.state.initialized) {
        if (g_chassis.state.fault_latched) {
            chassis_send_stop_once();
            return CHASSIS_SERVICE_STATUS_FAULT_LATCHED;
        }
        return CHASSIS_SERVICE_STATUS_NOT_INITIALIZED;
    }
    (void)fs_ia10b_maintain(&g_chassis.receiver);
    if (g_chassis.drive_rx_overflow) {
        uint32_t critical_state = stm32_critical_enter();
        g_chassis.drive_rx_overflow = false;
        stm32_critical_exit(critical_state);
        chassis_latch_fault(CHASSIS_FAULT_DRIVE_RECEIVE_OVERFLOW);
    }
    while (g_chassis.drive_rx_tail != g_chassis.drive_rx_head) {
        Stm32FdcanFrame frame;
        uint8_t i;
        uint32_t critical_state = stm32_critical_enter();
        frame.id = g_chassis.drive_rx_queue[g_chassis.drive_rx_tail].id;
        frame.id_type = g_chassis.drive_rx_queue[g_chassis.drive_rx_tail].id_type;
        frame.len = g_chassis.drive_rx_queue[g_chassis.drive_rx_tail].len;
        for (i = 0U; i < frame.len; ++i) {
            frame.data[i] = g_chassis.drive_rx_queue[g_chassis.drive_rx_tail].data[i];
        }
        g_chassis.drive_rx_tail =
            (uint8_t)((g_chassis.drive_rx_tail + 1U) % CHASSIS_DRIVE_RX_QUEUE_DEPTH);
        stm32_critical_exit(critical_state);
        (void)benmo_drive_motor_handle_feedback(&g_chassis.drive,
                                                frame.id,
                                                frame.data, frame.len);
    }
    if (g_chassis.state.fault_latched) {
        chassis_send_stop_once();
        return CHASSIS_SERVICE_STATUS_FAULT_LATCHED;
    }

    now = stm32_time_now_ms();
    if ((uint32_t)(now - g_chassis.last_update_ms) < CHASSIS_UPDATE_PERIOD_MS) {
        return CHASSIS_SERVICE_STATUS_OK;
    }
    g_chassis.last_update_ms = now;
    chassis_update_remote_command();
    if (steer_wheel_ik(&g_chassis.kinematics) != STEER_WHEEL_OK) {
        chassis_latch_fault(CHASSIS_FAULT_KINEMATICS);
        return CHASSIS_SERVICE_STATUS_KINEMATICS_ERROR;
    }
    if (steer_wheel_apply_legacy_57_correction(&g_chassis.kinematics) !=
        STEER_WHEEL_OK) {
        chassis_latch_fault(CHASSIS_FAULT_KINEMATICS);
        return CHASSIS_SERVICE_STATUS_KINEMATICS_ERROR;
    }

    for (i = 0U; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        int16_t rpm = (int16_t)(g_chassis.kinematics.control.wheels[i].wheel_omega *
                                CHASSIS_RAD_S_TO_RPM);
        (void)benmo_drive_motor_set_target_rpm(&g_chassis.drive,
                                               g_chassis.drive_ids[i], rpm);
        if (rs06_steer_motor_set_mode(&g_chassis.steer, g_chassis.steer_ids[i],
                                      RS06_STEER_MODE_PP) != RS06_STEER_STATUS_OK ||
            rs06_steer_motor_enable(&g_chassis.steer, g_chassis.steer_ids[i]) !=
                RS06_STEER_STATUS_OK ||
            rs06_steer_motor_set_position_target(
                &g_chassis.steer, g_chassis.steer_ids[i],
                g_chassis.kinematics.control.wheels[i].steer_angle) !=
                RS06_STEER_STATUS_OK) {
            steer_sends_ok = false;
        }
    }
    if (benmo_drive_motor_update(&g_chassis.drive) != BENMO_DRIVE_STATUS_OK ||
        benmo_drive_motor_request_next_feedback(&g_chassis.drive) !=
            BENMO_DRIVE_STATUS_OK) {
        drive_sends_ok = false;
    }
    if (!steer_sends_ok) {
        (void)chassis_record_send_result(false, CHASSIS_FAULT_STEER_TRANSMIT);
    } else if (!drive_sends_ok) {
        (void)chassis_record_send_result(false, CHASSIS_FAULT_DRIVE_TRANSMIT);
    } else {
        (void)chassis_record_send_result(true, CHASSIS_FAULT_NONE);
    }
    if (g_chassis.state.fault_latched) {
        return CHASSIS_SERVICE_STATUS_FAULT_LATCHED;
    }
    if (benmo_drive_motor_check_feedback(&g_chassis.drive) ==
        BENMO_DRIVE_STATUS_FEEDBACK_TIMEOUT) {
        chassis_latch_fault(CHASSIS_FAULT_DRIVE_FEEDBACK_TIMEOUT);
        return CHASSIS_SERVICE_STATUS_FAULT_LATCHED;
    }

    g_chassis.state.update_count++;
    if ((uint32_t)(now - g_chassis.last_log_ms) >= CHASSIS_LOG_PERIOD_MS) {
        g_chassis.last_log_ms = now;
        log_info("[遥控指令 x1000] vx=%ld vy=%ld wz=%ld",
                 (long)(g_chassis.state.command_vx * 1000.0F),
                 (long)(g_chassis.state.command_vy * 1000.0F),
                 (long)(g_chassis.state.command_wz * 1000.0F));
    }
    return (drive_sends_ok && steer_sends_ok)
               ? CHASSIS_SERVICE_STATUS_OK
               : CHASSIS_SERVICE_STATUS_DEVICE_ERROR;
}

ChassisServiceStatus chassis_service_stop(void)
{
    if (!g_chassis.drive.initialized && !g_chassis.steer.initialized) {
        return CHASSIS_SERVICE_STATUS_NOT_INITIALIZED;
    }
    g_chassis.stop_sent = false;
    g_chassis.fault_clear_armed = false;
    chassis_latch_fault(CHASSIS_FAULT_MANUAL_STOP);
    return CHASSIS_SERVICE_STATUS_OK;
}

ChassisServiceStatus chassis_service_fault_clear(void)
{
    uint8_t i;
    if (!g_chassis.state.initialized) {
        return CHASSIS_SERVICE_STATUS_NOT_INITIALIZED;
    }
    if (!g_chassis.state.fault_latched ||
        !chassis_remote_is_safe_for_fault_clear()) {
        return CHASSIS_SERVICE_STATUS_UNSAFE_STATE;
    }
    for (i = 0U; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        if (rs06_steer_motor_set_mode(&g_chassis.steer, g_chassis.steer_ids[i],
                                      RS06_STEER_MODE_PP) != RS06_STEER_STATUS_OK ||
            rs06_steer_motor_enable(&g_chassis.steer, g_chassis.steer_ids[i]) !=
                RS06_STEER_STATUS_OK ||
            rs06_steer_motor_set_position_target(&g_chassis.steer,
                                                  g_chassis.steer_ids[i], 0.0F) !=
                RS06_STEER_STATUS_OK) {
            return CHASSIS_SERVICE_STATUS_DEVICE_ERROR;
        }
    }
    (void)benmo_drive_motor_reset_feedback_monitor(&g_chassis.drive);
    g_chassis.consecutive_send_failures = 0U;
    g_chassis.stop_sent = false;
    g_chassis.fault_clear_armed = false;
    g_chassis.state.fault = CHASSIS_FAULT_NONE;
    g_chassis.state.fault_latched = false;
    chassis_set_velocity(0.0F, 0.0F, 0.0F);
    (void)log_info("[安全] fault cleared by explicit request");
    return CHASSIS_SERVICE_STATUS_OK;
}

ChassisServiceStatus chassis_service_get_state(ChassisServiceState* out)
{
    uint32_t critical_state;
    if (out == NULL) {
        return CHASSIS_SERVICE_STATUS_INVALID_PARAM;
    }
    critical_state = stm32_critical_enter();
    *out = g_chassis.state;
    stm32_critical_exit(critical_state);
    return CHASSIS_SERVICE_STATUS_OK;
}

const char* chassis_service_status_str(ChassisServiceStatus status)
{
    switch (status) {
        case CHASSIS_SERVICE_STATUS_OK: return "OK";
        case CHASSIS_SERVICE_STATUS_INVALID_PARAM: return "INVALID_PARAM";
        case CHASSIS_SERVICE_STATUS_NOT_INITIALIZED: return "NOT_INITIALIZED";
        case CHASSIS_SERVICE_STATUS_INIT_FAILED: return "INIT_FAILED";
        case CHASSIS_SERVICE_STATUS_DEVICE_ERROR: return "DEVICE_ERROR";
        case CHASSIS_SERVICE_STATUS_KINEMATICS_ERROR: return "KINEMATICS_ERROR";
        case CHASSIS_SERVICE_STATUS_FAULT_LATCHED: return "FAULT_LATCHED";
        case CHASSIS_SERVICE_STATUS_UNSAFE_STATE: return "UNSAFE_STATE";
        default: return "UNKNOWN";
    }
}

const char* chassis_service_fault_str(ChassisFault fault)
{
    switch (fault) {
        case CHASSIS_FAULT_NONE: return "NONE";
        case CHASSIS_FAULT_INITIALIZATION: return "INITIALIZATION";
        case CHASSIS_FAULT_DRIVE_TRANSMIT: return "DRIVE_TRANSMIT";
        case CHASSIS_FAULT_STEER_TRANSMIT: return "STEER_TRANSMIT";
        case CHASSIS_FAULT_DRIVE_RECEIVE_OVERFLOW: return "DRIVE_RECEIVE_OVERFLOW";
        case CHASSIS_FAULT_DRIVE_FEEDBACK_TIMEOUT: return "DRIVE_FEEDBACK_TIMEOUT";
        case CHASSIS_FAULT_KINEMATICS: return "KINEMATICS";
        case CHASSIS_FAULT_MANUAL_STOP: return "MANUAL_STOP";
        default: return "UNKNOWN";
    }
}
