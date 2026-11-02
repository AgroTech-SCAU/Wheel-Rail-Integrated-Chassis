/**
 * @file chassis_service.c
 * @brief 轮轨复合底盘服务实现
 */

#include "chassis_service.h"

#include "benmo_drive_motor.h"
#include "fs_ia10b.h"
#include "log.h"
#include "rs06_steer_motor.h"
#include "steer_home_control.h"
#include "steer_wheel_kinematics.h"
#include "stm32_board_port.h"
#include "stm32_fdcan_port.h"
#include "stm32_time_port.h"
#include "stm32_uart_port.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

// ! ========================= 宏 定 义 声 明 ========================= ! //

#define CHASSIS_LENGTH_M 0.725f
#define CHASSIS_WIDTH_M 0.730f
#define CHASSIS_WHEEL_RADIUS_M 0.0215f
#define CHASSIS_MAX_WHEEL_LINEAR_SPEED_M_S 0.45f
#define CHASSIS_UPDATE_PERIOD_MS 10u
#define CHASSIS_REMOTE_TIMEOUT_MS 200u
#define CHASSIS_DRIVE_FEEDBACK_TIMEOUT_MS 500u
#define CHASSIS_DRIVE_RX_QUEUE_CAP 8u
#define CHASSIS_SEND_FAILURE_LIMIT 3u
#define CHASSIS_RAD_S_TO_RPM 9.5492965855f
#define CHASSIS_RPM_TO_RAD_S 0.1047197551f
#define CHASSIS_STEER_HOME_KP 6.0f
#define CHASSIS_STEER_HOME_MAX_SPEED_RAD_S 1.2f
#define CHASSIS_STEER_HOME_DEADBAND_RAD 0.03f
#define CHASSIS_STEER_HOME_STABLE_CYCLES 8u
#define CHASSIS_STEER_HOME_TIMEOUT_MS 8000u
#define CHASSIS_STEER_FEEDBACK_MAX_AGE_MS 100u
#define CHASSIS_TASK_ZERO_DEADBAND_RAD 0.03f
#define CHASSIS_TASK_MAX_TIMEOUT_S 30.0f

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

typedef enum {
    CHASSIS_STEER_STARTUP_ENABLE = 0,
    CHASSIS_STEER_STARTUP_WAIT_FEEDBACK,
    CHASSIS_STEER_STARTUP_PREPARE_HOME,
    CHASSIS_STEER_STARTUP_HOME,
    CHASSIS_STEER_STARTUP_SWITCH_PP,
    CHASSIS_STEER_STARTUP_READY,
} ChassisSteerStartupState;

/**
 * @brief 底盘服务内部运行上下文
 */
typedef struct {
    FsIa10b receiver;
    BenmoDriveMotor drive;
    Rs06SteerMotor steer;
    SteerWheel kinematics;
    ChassisServiceConfig config;
    ChassisBoardInputs board_inputs;
    ChassisServiceState state;
    volatile Stm32FdcanFrame drive_rx_queue[CHASSIS_DRIVE_RX_QUEUE_CAP];
    volatile uint8_t drive_rx_head;
    volatile uint8_t drive_rx_tail;
    volatile bool drive_rx_overflow;
    uint8_t drive_ids[BENMO_DRIVE_MOTOR_COUNT];
    uint8_t steer_ids[BENMO_DRIVE_MOTOR_COUNT];
    float drive_speed_scale; /**< SWC 仅缩放本末 ID 1-4 的目标转速 */
    float last_steer_target[BENMO_DRIVE_MOTOR_COUNT];
    volatile float pending_steer_angle[BENMO_DRIVE_MOTOR_COUNT];
    volatile float pending_steer_velocity[BENMO_DRIVE_MOTOR_COUNT];
    volatile float pending_steer_torque[BENMO_DRIVE_MOTOR_COUNT];
    volatile float pending_steer_temperature[BENMO_DRIVE_MOTOR_COUNT];
    volatile uint8_t pending_steer_fault_bits[BENMO_DRIVE_MOTOR_COUNT];
    volatile uint8_t pending_steer_mode_status[BENMO_DRIVE_MOTOR_COUNT];
    volatile uint32_t pending_steer_feedback_ms[BENMO_DRIVE_MOTOR_COUNT];
    volatile uint32_t pending_steer_feedback_sequence[BENMO_DRIVE_MOTOR_COUNT];
    volatile bool pending_steer_feedback_valid[BENMO_DRIVE_MOTOR_COUNT];
    float steer_feedback_angle[BENMO_DRIVE_MOTOR_COUNT];
    float steer_feedback_raw_angle[BENMO_DRIVE_MOTOR_COUNT];
    float steer_feedback_velocity[BENMO_DRIVE_MOTOR_COUNT];
    float steer_feedback_torque[BENMO_DRIVE_MOTOR_COUNT];
    float steer_feedback_temperature[BENMO_DRIVE_MOTOR_COUNT];
    uint8_t steer_feedback_fault_bits[BENMO_DRIVE_MOTOR_COUNT];
    uint8_t steer_feedback_mode_status[BENMO_DRIVE_MOTOR_COUNT];
    uint32_t steer_feedback_ms[BENMO_DRIVE_MOTOR_COUNT];
    uint32_t steer_feedback_sequence[BENMO_DRIVE_MOTOR_COUNT];
    uint32_t steer_home_last_sequence[BENMO_DRIVE_MOTOR_COUNT];
    bool steer_feedback_valid[BENMO_DRIVE_MOTOR_COUNT];
    ChassisSteerStartupState steer_startup_state;
    uint8_t steer_startup_motor_index;
    uint8_t steer_startup_command_step;
    uint32_t steer_startup_wait_sequence;
    bool steer_startup_waiting_feedback;
    uint8_t steer_home_stable_cycles;
    uint32_t steer_home_start_ms;
    uint32_t last_update_ms;
    uint8_t consecutive_send_failures;
    uint32_t fault_clear_after_frame_count;
    bool stop_sent;
    bool fault_clear_armed;
    bool actuators_enabled;
    uint32_t session_id;
    uint64_t velocity_rx_us;
    bool velocity_seen;
    uint16_t velocity_valid_ms;
    float autonomous_vx, autonomous_vy, autonomous_wz;
    float manual_vx, manual_vy, manual_wz;
    uint32_t task_sequence;
    WrSteeringTaskState task;
    float task_start_angle[BENMO_DRIVE_MOTOR_COUNT];
    float task_park_raw_angle[BENMO_DRIVE_MOTOR_COUNT];
    float task_speed_rad_s;
    float task_max_angle_rad;
    uint64_t task_deadline_us;
    uint64_t task_timeout_us;
    uint8_t task_release_mask;
    uint64_t measured_at_us;
} ChassisServiceContext;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static bool chassis_drive_send(uint32_t id, const uint8_t* data, uint8_t len);
static bool chassis_steer_send(uint32_t id, const uint8_t* data, uint8_t len);
static bool chassis_log_write(const char* data, uint32_t len);
static void chassis_uart_rx_complete(void* context);
static void chassis_uart_error(void* context);
static void chassis_drive_rx(const Stm32FdcanFrame* frame, void* context);
static void chassis_steer_rx(const Stm32FdcanFrame* frame, void* context);
static float chassis_limit_float(float value, float minimum, float maximum);
static bool chassis_remote_switch_is(uint16_t value, uint16_t target);
static float chassis_remote_channel_to_norm(uint16_t value);
static float chassis_remote_channel_to_speed(uint16_t value, float maximum);
static RemoteSpeedLimit chassis_remote_speed_limit(uint16_t switch_value);
static void chassis_set_velocity(float vx, float vy, float wz);
static void chassis_update_remote_command(void);
static bool chassis_board_safe(void);
static void chassis_set_actuators_enabled(bool enabled);
static void chassis_cancel_task(uint8_t phase, WrResult result);
static bool chassis_task_is_running(void);
static bool chassis_run_task(uint64_t now_us);
static void chassis_update_feedback_velocity(uint32_t now);
static bool chassis_remote_is_safe_for_fault_clear(void);
static bool chassis_steer_id_to_index(uint8_t motor_id, uint8_t* out_index);
static bool chassis_steer_feedback_ready(void);
static bool chassis_steer_feedback_fresh(uint32_t now);
static bool chassis_steer_run_status(uint32_t now);
static bool chassis_drive_feedback_ready(uint32_t now);
static bool chassis_steer_feedback_safe(void);
static bool chassis_take_steer_feedback_snapshot(void);
static bool chassis_run_steer_home(uint32_t now, bool* steer_sends_ok);
static int8_t chassis_prepare_next_steering_motor(bool velocity_mode);
static void chassis_seed_steer_targets_from_feedback(void);
static void chassis_send_stop_once(void);
static void chassis_latch_fault(ChassisFault fault);
static bool chassis_record_send_result(bool success, ChassisFault fault);
static bool chassis_initialize_steering_step(void);

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
    volatile Stm32FdcanFrame* slot;
    uint8_t next;
    uint8_t i;
    if(frame == NULL || self == NULL ||
       frame->id_type != STM32_FDCAN_ID_STANDARD ||
       frame->len != BENMO_DRIVE_MOTOR_CAN_DATA_LEN) {
        return;
    }
    next = (uint8_t)((self->drive_rx_head + 1u) % CHASSIS_DRIVE_RX_QUEUE_CAP);
    if(next == self->drive_rx_tail) {
        self->drive_rx_overflow = true;
        return;
    }
    slot = &self->drive_rx_queue[self->drive_rx_head];
    slot->id = frame->id;
    slot->id_type = frame->id_type;
    slot->len = frame->len;
    for(i = 0u; i < frame->len; ++i) {
        slot->data[i] = frame->data[i];
    }
    self->drive_rx_head = next;
}

static void chassis_steer_rx(const Stm32FdcanFrame* frame, void* context) {
    ChassisServiceContext* self = (ChassisServiceContext*)context;
    Rs06SteerMotorFeedback feedback;
    uint8_t index;
    if(frame == NULL || self == NULL ||
       frame->id_type != STM32_FDCAN_ID_EXTENDED ||
       rs06_steer_motor_parse_feedback(&self->steer, frame->id, frame->data,
                                       frame->len, &feedback) !=
           RS06_STEER_STATUS_OK ||
       !chassis_steer_id_to_index(feedback.motor_id, &index)) {
        return;
    }
    self->pending_steer_angle[index] = feedback.angle_rad;
    self->pending_steer_velocity[index] = feedback.velocity_rad_s;
    self->pending_steer_torque[index] = feedback.torque_nm;
    self->pending_steer_temperature[index] = feedback.temperature_c;
    self->pending_steer_fault_bits[index] = feedback.fault_bits;
    self->pending_steer_mode_status[index] = feedback.mode_status;
    self->pending_steer_feedback_ms[index] = stm32_time_now_ms();
    self->pending_steer_feedback_sequence[index]++;
    self->pending_steer_feedback_valid[index] = true;
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
    s_chassis.manual_vx = s_chassis.manual_vy = s_chassis.manual_wz = 0.0f;
    s_chassis.drive_speed_scale = 1.0f;
    if(fs_ia10b_get_data(&s_chassis.receiver, &remote) != FS_IA10B_STATUS_OK ||
       !fs_ia10b_is_online(&s_chassis.receiver, CHASSIS_REMOTE_TIMEOUT_MS)) {
        s_chassis.state.remote_online = false;
        s_chassis.state.remote_enabled = false;
        if(s_chassis.state.source == WR_SOURCE_MANUAL) {
            s_chassis.state.source = WR_SOURCE_NONE;
            s_chassis.state.enabled = false;
            chassis_set_actuators_enabled(false);
        }
        return;
    }
    s_chassis.state.remote_online = true;
    if(remote.channel[REMOTE_CH_VRB] > REMOTE_ENABLE_THRESHOLD) {
        s_chassis.state.remote_enabled = false;
        if(s_chassis.state.source == WR_SOURCE_MANUAL) {
            s_chassis.state.source = WR_SOURCE_NONE;
            s_chassis.state.enabled = false;
            chassis_set_actuators_enabled(false);
        }
        return;
    }
    s_chassis.state.remote_enabled = true;
    if(s_chassis.state.source != WR_SOURCE_MANUAL) {
        chassis_set_actuators_enabled(false);
        s_chassis.session_id = 0u;
        s_chassis.state.source = WR_SOURCE_MANUAL;
        s_chassis.state.mode = WR_MODE_FREE;
        s_chassis.state.enabled = true;
        s_chassis.autonomous_vx = s_chassis.autonomous_vy = s_chassis.autonomous_wz = 0.0f;
        s_chassis.velocity_rx_us = 0u;
        s_chassis.velocity_seen = false;
    }
    /* iBUS 1000/1500/2000 分别对应 SWC 负/中/正档 */
    if(chassis_remote_switch_is(remote.channel[FS_IA10B_CH_SWC],
                                REMOTE_SWITCH_HIGH)) {
        s_chassis.drive_speed_scale = 0.25f;
    }
    else if(chassis_remote_switch_is(remote.channel[FS_IA10B_CH_SWC],
                                     REMOTE_SWITCH_LOW)) {
        s_chassis.drive_speed_scale = 1.0f;
    }
    limit = chassis_remote_speed_limit(remote.channel[REMOTE_CH_SWB]);
    s_chassis.manual_vx = -chassis_remote_channel_to_speed(
        remote.channel[REMOTE_CH_RIGHT_Y], limit.max_vx);
    s_chassis.manual_vy = -chassis_remote_channel_to_speed(
        remote.channel[REMOTE_CH_RIGHT_X], limit.max_vy);
    s_chassis.manual_wz = -chassis_remote_channel_to_speed(
        remote.channel[REMOTE_CH_LEFT_X], limit.max_wz);
}

static bool chassis_steer_id_to_index(uint8_t motor_id, uint8_t* out_index) {
    uint8_t i;
    if(out_index == NULL) {
        return false;
    }
    for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        if(s_chassis.steer_ids[i] == motor_id) {
            *out_index = i;
            return true;
        }
    }
    return false;
}

static bool chassis_steer_feedback_ready(void) {
    uint8_t i;
    for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        if(!s_chassis.steer_feedback_valid[i]) {
            return false;
        }
    }
    return true;
}

static bool chassis_run_steer_home(uint32_t now, bool* steer_sends_ok) {
    uint8_t i;
    uint8_t done_count = 0u;
    bool all_samples_new = true;
    bool send_ok = true;

    if(steer_sends_ok == NULL) {
        return false;
    }
    *steer_sends_ok = true;

    if((uint32_t)(now - s_chassis.steer_home_start_ms) >=
       CHASSIS_STEER_HOME_TIMEOUT_MS) {
        for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
            (void)rs06_steer_motor_set_velocity_target(
                &s_chassis.steer, s_chassis.steer_ids[i], 0.0f);
            (void)rs06_steer_motor_stop(&s_chassis.steer,
                                        s_chassis.steer_ids[i]);
        }
        chassis_latch_fault(CHASSIS_FAULT_STEER_FEEDBACK_TIMEOUT);
        *steer_sends_ok = false;
        return false;
    }
    if(!chassis_steer_feedback_fresh(now)) {
        for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
            if(rs06_steer_motor_set_velocity_target(
                   &s_chassis.steer, s_chassis.steer_ids[i], 0.0f) !=
               RS06_STEER_STATUS_OK) {
                *steer_sends_ok = false;
            }
        }
        return false;
    }

    for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        const float feedback_angle = s_chassis.steer_feedback_angle[i];
        if(s_chassis.steer_feedback_sequence[i] ==
           s_chassis.steer_home_last_sequence[i]) {
            all_samples_new = false;
        }
        float speed_cmd = steer_home_speed_command(
            feedback_angle, CHASSIS_STEER_HOME_KP,
            CHASSIS_STEER_HOME_MAX_SPEED_RAD_S);
        if(fabsf(speed_cmd) <=
           CHASSIS_STEER_HOME_KP * CHASSIS_STEER_HOME_DEADBAND_RAD) {
            speed_cmd = 0.0f;
            done_count++;
        }

        send_ok = rs06_steer_motor_set_velocity_target(
                      &s_chassis.steer, s_chassis.steer_ids[i], speed_cmd) ==
                  RS06_STEER_STATUS_OK;
        if(!send_ok) {
            *steer_sends_ok = false;
        }
    }

    if(all_samples_new && done_count == BENMO_DRIVE_MOTOR_COUNT) {
        if(s_chassis.steer_home_stable_cycles < 0xFFu) {
            s_chassis.steer_home_stable_cycles++;
        }
    }
    else if(all_samples_new) {
        s_chassis.steer_home_stable_cycles = 0u;
    }

    if(all_samples_new) {
        for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
            s_chassis.steer_home_last_sequence[i] =
                s_chassis.steer_feedback_sequence[i];
        }
    }

    if(s_chassis.steer_home_stable_cycles >= CHASSIS_STEER_HOME_STABLE_CYCLES) {
        s_chassis.steer_startup_motor_index = 0u;
        s_chassis.steer_startup_waiting_feedback = false;
        s_chassis.steer_home_start_ms = now;
        s_chassis.steer_startup_state = CHASSIS_STEER_STARTUP_SWITCH_PP;
        return true;
    }

    return false;
}

static bool chassis_steer_feedback_fresh(uint32_t now) {
    uint8_t i;
    if(!chassis_steer_feedback_ready()) {
        return false;
    }
    for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        if(!steer_home_feedback_timestamp_is_fresh(
               now, s_chassis.steer_feedback_ms[i],
               CHASSIS_STEER_FEEDBACK_MAX_AGE_MS)) {
            return false;
        }
    }
    return true;
}

static bool chassis_steer_run_status(uint32_t now) {
    uint8_t i;
    if(!chassis_steer_feedback_fresh(now)) return false;
    for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        if(s_chassis.steer_feedback_mode_status[i] !=
               RS06_STEER_FEEDBACK_MODE_RUN ||
           s_chassis.steer_feedback_fault_bits[i] != 0u) return false;
    }
    return true;
}

static bool chassis_drive_feedback_ready(uint32_t now) {
    uint8_t i;
    for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        BenmoDriveMotorFeedback feedback;
        if(benmo_drive_motor_get_feedback(&s_chassis.drive, s_chassis.drive_ids[i],
                                          &feedback) != BENMO_DRIVE_STATUS_OK ||
           !feedback.valid || feedback.error_code != 0u ||
           (uint32_t)(now - feedback.last_update_ms) >=
               CHASSIS_DRIVE_FEEDBACK_TIMEOUT_MS)
            return false;
    }
    return true;
}

static bool chassis_take_steer_feedback_snapshot(void) {
    uint32_t critical_state;
    uint8_t i;
    bool changed = false;
    critical_state = stm32_critical_enter();
    for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        if(s_chassis.pending_steer_feedback_valid[i] &&
           s_chassis.pending_steer_feedback_sequence[i] !=
               s_chassis.steer_feedback_sequence[i]) {
            s_chassis.steer_feedback_angle[i] =
                steer_home_normalize_feedback(s_chassis.pending_steer_angle[i]);
            s_chassis.steer_feedback_raw_angle[i] =
                s_chassis.pending_steer_angle[i];
            s_chassis.steer_feedback_velocity[i] =
                s_chassis.pending_steer_velocity[i];
            s_chassis.steer_feedback_torque[i] =
                s_chassis.pending_steer_torque[i];
            s_chassis.steer_feedback_temperature[i] =
                s_chassis.pending_steer_temperature[i];
            s_chassis.steer_feedback_fault_bits[i] =
                s_chassis.pending_steer_fault_bits[i];
            s_chassis.steer_feedback_mode_status[i] =
                s_chassis.pending_steer_mode_status[i];
            s_chassis.steer_feedback_ms[i] =
                s_chassis.pending_steer_feedback_ms[i];
            s_chassis.steer_feedback_sequence[i] =
                s_chassis.pending_steer_feedback_sequence[i];
            s_chassis.steer_feedback_valid[i] = true;
            changed = true;
        }
    }
    stm32_critical_exit(critical_state);
    for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        if(s_chassis.steer_feedback_valid[i]) {
            s_chassis.kinematics.state.cur_wheels[i].steer_angle =
                s_chassis.steer_feedback_angle[i];
        }
    }
    return changed;
}

static int8_t chassis_prepare_next_steering_motor(bool velocity_mode) {
    const uint8_t i = s_chassis.steer_startup_motor_index;
    Rs06SteerMotorStatus status;

    if(i >= BENMO_DRIVE_MOTOR_COUNT) {
        return 1;
    }
    if(s_chassis.steer_startup_waiting_feedback) {
        if(s_chassis.steer_feedback_sequence[i] ==
           s_chassis.steer_startup_wait_sequence) {
            return 0;
        }
        s_chassis.steer_startup_waiting_feedback = false;
        s_chassis.steer_startup_motor_index++;
        s_chassis.steer_startup_command_step = 0u;
        return s_chassis.steer_startup_motor_index >= BENMO_DRIVE_MOTOR_COUNT
                   ? 1
                   : 0;
    }
    switch(s_chassis.steer_startup_command_step) {
        case 0u:
            status = rs06_steer_motor_stop(&s_chassis.steer,
                                           s_chassis.steer_ids[i]);
            break;
        case 1u:
            status = rs06_steer_motor_set_mode(
                &s_chassis.steer, s_chassis.steer_ids[i],
                velocity_mode ? RS06_STEER_MODE_VELOCITY : RS06_STEER_MODE_PP);
            break;
        case 2u:
            status = velocity_mode
                ? rs06_steer_motor_set_velocity_target(
                      &s_chassis.steer, s_chassis.steer_ids[i], 0.0f)
                : rs06_steer_motor_set_raw_position_target(
                      &s_chassis.steer, s_chassis.steer_ids[i],
                      steer_home_raw_position_target(
                          0.0f, s_chassis.steer_feedback_raw_angle[i]));
            break;
        default: {
            const uint32_t critical_state = stm32_critical_enter();
            s_chassis.steer_startup_wait_sequence =
                s_chassis.pending_steer_feedback_sequence[i];
            stm32_critical_exit(critical_state);
            status = rs06_steer_motor_enable(&s_chassis.steer,
                                             s_chassis.steer_ids[i]);
            break;
        }
    }
    if(status != RS06_STEER_STATUS_OK) {
        return -1;
    }
    if(s_chassis.steer_startup_command_step < 3u) {
        s_chassis.steer_startup_command_step++;
        return 0;
    }
    s_chassis.steer_startup_waiting_feedback = true;
    return 0;
}

static void chassis_seed_steer_targets_from_feedback(void) {
    uint8_t i;
    if(!chassis_steer_feedback_ready()) {
        return;
    }
    for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        s_chassis.last_steer_target[i] = 0.0f;
        s_chassis.kinematics.state.cur_wheels[i].steer_angle =
            s_chassis.steer_feedback_angle[i];
    }
}

static bool chassis_board_safe(void) {
    return (!s_chassis.config.require_estop || s_chassis.board_inputs.estop_valid) &&
           !(s_chassis.board_inputs.estop_valid && s_chassis.board_inputs.physical_estop);
}

static void chassis_cancel_task(uint8_t phase, WrResult result) {
    uint8_t mask = (uint8_t)(s_chassis.task.active_mask |
                             s_chassis.task.holding_mask |
                             s_chassis.task_release_mask);
    uint8_t i;
    if(mask == 0u) return;
    for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        if((mask & (uint8_t)(1u << i)) != 0u) {
            (void)rs06_steer_motor_set_velocity_target(&s_chassis.steer,
                                                        s_chassis.steer_ids[i], 0.0f);
            (void)rs06_steer_motor_stop(&s_chassis.steer, s_chassis.steer_ids[i]);
        }
    }
    s_chassis.task.active_mask = 0u;
    s_chassis.task.holding_mask = 0u;
    s_chassis.task_release_mask = 0u;
    s_chassis.task.phase = phase;
    s_chassis.task.result = result;
}

static bool chassis_task_is_running(void) {
    return (s_chassis.task.active_mask | s_chassis.task.holding_mask |
            s_chassis.task_release_mask) != 0u;
}

static void chassis_set_actuators_enabled(bool enabled) {
    uint8_t i;
    if(enabled == s_chassis.actuators_enabled) return;
    if(!enabled) {
        chassis_cancel_task(WR_TASK_CANCELLED, WR_RESULT_UNAUTHORIZED);
        s_chassis.stop_sent = false;
        chassis_send_stop_once();
        s_chassis.actuators_enabled = false;
        s_chassis.state.motor_power_ready = false;
        s_chassis.steer_startup_state = CHASSIS_STEER_STARTUP_WAIT_FEEDBACK;
        s_chassis.steer_home_start_ms = stm32_time_now_ms();
        s_chassis.steer_startup_motor_index = 0u;
        s_chassis.steer_startup_command_step = 0u;
        s_chassis.steer_startup_waiting_feedback = false;
        for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
            s_chassis.steer_feedback_valid[i] = false;
        }
        (void)benmo_drive_motor_reset_feedback_monitor(&s_chassis.drive);
        return;
    }
    s_chassis.actuators_enabled = true;
    s_chassis.stop_sent = false;
    s_chassis.steer_home_start_ms = stm32_time_now_ms();
    s_chassis.steer_startup_state = CHASSIS_STEER_STARTUP_ENABLE;
    s_chassis.steer_startup_motor_index = 0u;
    s_chassis.steer_startup_command_step = 0u;
}

static void chassis_task_safety_stop(uint8_t module, uint8_t phase,
                                     WrResult result) {
    const uint8_t bit = (uint8_t)(1u << module);
    s_chassis.task.safety_stopped_mask |= bit;
    s_chassis.task.active_mask = 0u;
    s_chassis.task.holding_mask = 0u;
    s_chassis.task_release_mask = 0u;
    s_chassis.task.phase = phase;
    s_chassis.task.result = result;
    s_chassis.autonomous_vx = s_chassis.autonomous_vy =
        s_chassis.autonomous_wz = 0.0f;
    s_chassis.velocity_rx_us = 0u;
    s_chassis.velocity_seen = false;
    s_chassis.state.enabled = false;
    s_chassis.state.motor_power_ready = false;
    chassis_latch_fault(CHASSIS_FAULT_TASK_SAFETY);
}

static bool chassis_run_task(uint64_t now_us) {
    uint8_t i;
    bool send_ok = true;
    const uint32_t now_ms = stm32_time_now_ms();
    for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        const uint8_t bit = (uint8_t)(1u << i);
        const bool moving = (s_chassis.task.active_mask & bit) != 0u;
        float speed;
        if(!moving && (s_chassis.task.holding_mask & bit) == 0u) {
            /* A parameter write returns Type 2 feedback even when motor active
             * reporting is disabled; preserve the parked wheel's target. */
            if(rs06_steer_motor_set_raw_position_target(
                   &s_chassis.steer, s_chassis.steer_ids[i],
                   s_chassis.task_park_raw_angle[i]) != RS06_STEER_STATUS_OK)
                send_ok = false;
            continue;
        }
        if(now_us >= s_chassis.task_deadline_us) {
            chassis_task_safety_stop(i, WR_TASK_TIMEOUT, WR_RESULT_TIMEOUT);
            return false;
        }
        if(!s_chassis.steer_feedback_valid[i] ||
           !steer_home_feedback_timestamp_is_fresh(
               now_ms, s_chassis.steer_feedback_ms[i],
               CHASSIS_STEER_FEEDBACK_MAX_AGE_MS) ||
           !isfinite(s_chassis.steer_feedback_torque[i]) ||
           !isfinite(s_chassis.steer_feedback_temperature[i]) ||
           fabsf(s_chassis.steer_feedback_torque[i]) >=
               s_chassis.config.probe_hard_torque_nm ||
           s_chassis.steer_feedback_temperature[i] >=
               s_chassis.config.probe_max_temperature_c ||
           s_chassis.steer_feedback_angle[i] <
               s_chassis.config.probe_min_angle[i] ||
           s_chassis.steer_feedback_angle[i] >
               s_chassis.config.probe_max_angle[i] ||
           (moving && (s_chassis.task_release_mask & bit) == 0u &&
            fabsf(s_chassis.steer_feedback_angle[i] -
                  s_chassis.task_start_angle[i]) >= s_chassis.task_max_angle_rad)) {
            chassis_task_safety_stop(i, WR_TASK_SAFETY_STOPPED,
                                     WR_RESULT_SAFETY_STOP);
            return false;
        }
        if(!moving) {
            if(rs06_steer_motor_set_raw_position_target(
                   &s_chassis.steer, s_chassis.steer_ids[i],
                   s_chassis.task_park_raw_angle[i]) != RS06_STEER_STATUS_OK)
                send_ok = false;
            continue;
        }
        if((s_chassis.task_release_mask & bit) != 0u) {
            if(fabsf(s_chassis.steer_feedback_angle[i]) <=
               CHASSIS_TASK_ZERO_DEADBAND_RAD) {
                if(rs06_steer_motor_set_velocity_target(
                       &s_chassis.steer, s_chassis.steer_ids[i], 0.0f) !=
                       RS06_STEER_STATUS_OK ||
                   rs06_steer_motor_prepare_pp_raw(
                       &s_chassis.steer, s_chassis.steer_ids[i],
                       s_chassis.steer_feedback_raw_angle[i]) != RS06_STEER_STATUS_OK)
                    send_ok = false;
                s_chassis.task.active_mask &= (uint8_t)~bit;
                s_chassis.task_release_mask &= (uint8_t)~bit;
                s_chassis.task.completed_mask |= bit;
                s_chassis.task_park_raw_angle[i] = s_chassis.steer_feedback_raw_angle[i];
                continue;
            }
            speed = s_chassis.steer_feedback_angle[i] > 0.0f
                        ? -s_chassis.task_speed_rad_s : s_chassis.task_speed_rad_s;
        }
        else speed = s_chassis.task_speed_rad_s * s_chassis.config.probe_direction[i];
        if(rs06_steer_motor_set_velocity_target(
               &s_chassis.steer, s_chassis.steer_ids[i], speed) !=
           RS06_STEER_STATUS_OK) send_ok = false;
    }
    if(s_chassis.task.active_mask == 0u &&
       s_chassis.task.phase == WR_TASK_EXECUTING) {
        if(s_chassis.task.holding_mask != 0u) s_chassis.task.phase = WR_TASK_HOLDING;
        else s_chassis.task.phase = WR_TASK_COMPLETED;
    }
    return send_ok;
}

static void chassis_update_feedback_velocity(uint32_t now) {
    uint8_t i;
    uint32_t oldest_age_ms = 0u;
    s_chassis.kinematics.model.wheel_radius =
        s_chassis.state.mode == WR_MODE_ON_RAIL &&
        s_chassis.config.rail_radius_m > 0.0f
            ? s_chassis.config.rail_radius_m : s_chassis.config.model.wheel_radius;
    for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        BenmoDriveMotorFeedback feedback;
        if(!s_chassis.steer_feedback_valid[i] ||
           !steer_home_feedback_timestamp_is_fresh(
               now, s_chassis.steer_feedback_ms[i], CHASSIS_STEER_FEEDBACK_MAX_AGE_MS) ||
           benmo_drive_motor_get_feedback(&s_chassis.drive, s_chassis.drive_ids[i],
                                          &feedback) != BENMO_DRIVE_STATUS_OK ||
           !feedback.valid ||
           (uint32_t)(now - feedback.last_update_ms) >=
               CHASSIS_DRIVE_FEEDBACK_TIMEOUT_MS) {
            s_chassis.kinematics.state.cur_vx = 0.0f;
            s_chassis.kinematics.state.cur_vy = 0.0f;
            s_chassis.kinematics.state.cur_wz = 0.0f;
            s_chassis.measured_at_us = 0u;
            return;
        }
        s_chassis.kinematics.state.cur_wheels[i].wheel_omega =
            feedback.speed_rpm * CHASSIS_RPM_TO_RAD_S;
        s_chassis.kinematics.state.cur_wheels[i].steer_angle =
            s_chassis.steer_feedback_angle[i];
        if((uint32_t)(now - feedback.last_update_ms) > oldest_age_ms)
            oldest_age_ms = (uint32_t)(now - feedback.last_update_ms);
        if((uint32_t)(now - s_chassis.steer_feedback_ms[i]) > oldest_age_ms)
            oldest_age_ms = (uint32_t)(now - s_chassis.steer_feedback_ms[i]);
    }
    if(steer_wheel_fk(&s_chassis.kinematics) == STEER_WHEEL_OK) {
        const uint64_t current_us = stm32_time_now_us();
        const uint64_t age_us = (uint64_t)oldest_age_ms * 1000u;
        s_chassis.measured_at_us = current_us >= age_us ? current_us - age_us : 0u;
    }
}

static bool chassis_steer_feedback_safe(void) {
    uint8_t i;
    for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        if(!s_chassis.steer_feedback_valid[i] ||
           !isfinite(s_chassis.steer_feedback_raw_angle[i]) ||
           fabsf(s_chassis.steer_feedback_raw_angle[i]) >
               RS06_STEER_POSITION_MAX_RAD ||
           fabsf(s_chassis.steer_feedback_angle[i]) >
               RS06_STEER_SAFE_POSITION_RAD) {
            return false;
        }
    }
    return true;
}

static bool chassis_remote_is_safe_for_fault_clear(void) {
    FsIa10bData remote;
    (void)fs_ia10b_maintain(&s_chassis.receiver);
    if(fs_ia10b_get_data(&s_chassis.receiver, &remote) != FS_IA10B_STATUS_OK ||
       !fs_ia10b_is_online(&s_chassis.receiver, CHASSIS_REMOTE_TIMEOUT_MS)) {
        s_chassis.state.remote_online = false;
        s_chassis.state.remote_enabled = false;
        return true;
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
    if(first_fault) {
        FsIa10bData remote;
        if(fs_ia10b_get_data(&s_chassis.receiver, &remote) ==
           FS_IA10B_STATUS_OK) {
            s_chassis.fault_clear_after_frame_count = remote.frame_count;
            s_chassis.fault_clear_armed = true;
        }
        else s_chassis.fault_clear_armed = false;
    }
    chassis_set_velocity(0.0f, 0.0f, 0.0f);
    chassis_send_stop_once();
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

static bool chassis_initialize_steering_step(void) {
    const uint8_t i = s_chassis.steer_startup_motor_index;
    Rs06SteerMotorStatus status;
    if(i >= BENMO_DRIVE_MOTOR_COUNT) {
        s_chassis.steer_startup_state = CHASSIS_STEER_STARTUP_WAIT_FEEDBACK;
        s_chassis.steer_home_start_ms = stm32_time_now_ms();
        return true;
    }
    switch(s_chassis.steer_startup_command_step) {
        case 0u:
            status = rs06_steer_motor_stop(&s_chassis.steer, s_chassis.steer_ids[i]);
            break;
        case 1u:
            status = rs06_steer_motor_set_mode(&s_chassis.steer,
                                               s_chassis.steer_ids[i],
                                               RS06_STEER_MODE_VELOCITY);
            break;
        case 2u:
            status = rs06_steer_motor_set_velocity_target(&s_chassis.steer,
                                                           s_chassis.steer_ids[i],
                                                           0.0f);
            break;
        default:
            status = rs06_steer_motor_enable(&s_chassis.steer,
                                             s_chassis.steer_ids[i]);
            break;
    }
    if(status != RS06_STEER_STATUS_OK) return false;
    s_chassis.steer_startup_command_step++;
    if(s_chassis.steer_startup_command_step >= 4u) {
        s_chassis.steer_startup_command_step = 0u;
        s_chassis.steer_startup_motor_index++;
    }
    return true;
}

static void chassis_log_initialized(void) {
    (void)log_info("system initialized");
}

// ! ========================= 接 口 函 数 实 现 ========================= ! //

ChassisServiceConfig chassis_service_default_config(void) {
    const ChassisServiceConfig config = {
        .model = {
            .length = CHASSIS_LENGTH_M,
            .width = CHASSIS_WIDTH_M,
            .wheel_radius = CHASSIS_WHEEL_RADIUS_M,
            .max_wheel_linear_speed = CHASSIS_MAX_WHEEL_LINEAR_SPEED_M_S,
        },
        .yaw_bias = { 0 },
        .rail_radius_m = 0.0f,
        .drive_direction = { 1, 1, -1, -1 },
        .steer_direction = { -1, 1, -1, 1 },
        .probe_direction = { 0, 0, 0, 0 },
        .probe_min_angle = { 0.0f, 0.0f, 0.0f, 0.0f },
        .probe_max_angle = { 0.0f, 0.0f, 0.0f, 0.0f },
        .probe_hard_torque_nm = 0.0f,
        .probe_max_speed_rad_s = 0.0f,
        .probe_max_temperature_c = 0.0f,
        .require_estop = true,
        .legacy_steer_57_correction = true,
    };
    return config;
}

ChassisServiceStatus chassis_service_init(void) {
    const ChassisServiceConfig config = chassis_service_default_config();
    return chassis_service_init_with_config(&config);
}

ChassisServiceStatus chassis_service_init_with_config(
    const ChassisServiceConfig* config) {
    FsIa10bConfig receiver_config;
    BenmoDriveMotorConfig drive_config;
    Rs06SteerMotorConfig steer_config;
    LogConfig log_config;
    uint8_t i;

    if(config == NULL || !isfinite(config->yaw_bias.k_vx) ||
       !isfinite(config->yaw_bias.k_vy) ||
       !isfinite(config->yaw_bias.v_deadband) ||
       config->yaw_bias.v_deadband < 0.0f ||
       !isfinite(config->rail_radius_m) || config->rail_radius_m < 0.0f ||
       !isfinite(config->probe_hard_torque_nm) || config->probe_hard_torque_nm < 0.0f ||
       !isfinite(config->probe_max_speed_rad_s) ||
       config->probe_max_speed_rad_s < 0.0f ||
       config->probe_max_speed_rad_s > RS06_STEER_VELOCITY_MAX_RAD_S ||
       !isfinite(config->probe_max_temperature_c) || config->probe_max_temperature_c < 0.0f) {
        return CHASSIS_SERVICE_STATUS_INVALID_PARAM;
    }
    for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        if((config->drive_direction[i] != 1 && config->drive_direction[i] != -1) ||
           (config->steer_direction[i] != 1 && config->steer_direction[i] != -1) ||
           (config->probe_direction[i] != 0 && config->probe_direction[i] != 1 &&
            config->probe_direction[i] != -1) ||
           !isfinite(config->probe_min_angle[i]) ||
           !isfinite(config->probe_max_angle[i]) ||
           config->probe_min_angle[i] < -RS06_STEER_SAFE_POSITION_RAD ||
           config->probe_max_angle[i] > RS06_STEER_SAFE_POSITION_RAD ||
           config->probe_min_angle[i] > config->probe_max_angle[i]) {
            return CHASSIS_SERVICE_STATUS_INVALID_PARAM;
        }
    }

    memset(&s_chassis, 0, sizeof(s_chassis));
    s_chassis.config = *config;
    s_chassis.state.source = WR_SOURCE_NONE;
    s_chassis.state.mode = WR_MODE_FREE;
    s_chassis.task.phase = WR_TASK_IDLE;
    for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        s_chassis.drive_ids[i] = (uint8_t)(i + 1u);
        s_chassis.steer_ids[i] = (uint8_t)(i + 5u);
    }
    if(steer_wheel_init(&s_chassis.kinematics, config->model) != STEER_WHEEL_OK) {
        chassis_latch_fault(CHASSIS_FAULT_INITIALIZATION);
        return CHASSIS_SERVICE_STATUS_INIT_FAILED;
    }
    s_chassis.kinematics.yaw_bias = config->yaw_bias;

    stm32_board_receiver_power_enable();
    if(!stm32_fdcan_port_register_rx_callback(STM32_FDCAN_BUS_DRIVE,
                                              chassis_drive_rx, &s_chassis) ||
       !stm32_fdcan_port_init(STM32_FDCAN_BUS_DRIVE)) {
        chassis_latch_fault(CHASSIS_FAULT_INITIALIZATION);
        return CHASSIS_SERVICE_STATUS_INIT_FAILED;
    }
    drive_config.ops = &s_drive_ops;
    drive_config.feedback_timeout_ms = CHASSIS_DRIVE_FEEDBACK_TIMEOUT_MS;
    if(benmo_drive_motor_init(&s_chassis.drive, &drive_config) !=
           BENMO_DRIVE_STATUS_OK ||
       benmo_drive_motor_set_directions(&s_chassis.drive,
                                        config->drive_direction) !=
           BENMO_DRIVE_STATUS_OK) {
        chassis_latch_fault(CHASSIS_FAULT_INITIALIZATION);
        return CHASSIS_SERVICE_STATUS_INIT_FAILED;
    }
    if(benmo_drive_motor_stop_all(&s_chassis.drive) != BENMO_DRIVE_STATUS_OK) {
        chassis_latch_fault(CHASSIS_FAULT_INITIALIZATION);
        return CHASSIS_SERVICE_STATUS_INIT_FAILED;
    }

    steer_config.ops = &s_steer_ops;
    steer_config.host_id = RS06_STEER_HOST_ID;
    if(rs06_steer_motor_init(&s_chassis.steer, &steer_config) !=
           RS06_STEER_STATUS_OK ||
       rs06_steer_motor_set_directions(&s_chassis.steer,
                                       config->steer_direction) !=
           RS06_STEER_STATUS_OK ||
       !stm32_fdcan_port_register_rx_callback(STM32_FDCAN_BUS_STEER,
                                              chassis_steer_rx, &s_chassis) ||
       !stm32_fdcan_port_init(STM32_FDCAN_BUS_STEER)) {
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
    s_chassis.stop_sent = false;
    chassis_send_stop_once();
    s_chassis.steer_startup_state = CHASSIS_STEER_STARTUP_WAIT_FEEDBACK;
    s_chassis.steer_startup_motor_index = 0u;
    s_chassis.steer_startup_waiting_feedback = false;
    s_chassis.steer_home_stable_cycles = 0u;
    s_chassis.steer_home_start_ms = stm32_time_now_ms();
    chassis_log_initialized();
    s_chassis.last_update_ms = stm32_time_now_ms();
    (void)benmo_drive_motor_reset_feedback_monitor(&s_chassis.drive);
    return CHASSIS_SERVICE_STATUS_OK;
}

ChassisServiceStatus chassis_service_update(void) {
    uint32_t now;
    uint64_t now_us;
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
    while(s_chassis.drive_rx_tail != s_chassis.drive_rx_head) {
        Stm32FdcanFrame frame;
        uint8_t i;
        uint32_t critical_state = stm32_critical_enter();
        const volatile Stm32FdcanFrame* slot =
            &s_chassis.drive_rx_queue[s_chassis.drive_rx_tail];
        frame.id = slot->id;
        frame.id_type = slot->id_type;
        frame.len = slot->len;
        for(i = 0u; i < frame.len; ++i) {
            frame.data[i] = slot->data[i];
        }
        s_chassis.drive_rx_tail = (uint8_t)((s_chassis.drive_rx_tail + 1u) %
                                             CHASSIS_DRIVE_RX_QUEUE_CAP);
        stm32_critical_exit(critical_state);
        (void)benmo_drive_motor_handle_feedback(&s_chassis.drive, frame.id,
                                                frame.data, frame.len);
    }
    if(s_chassis.drive_rx_overflow) {
        s_chassis.drive_rx_overflow = false;
        chassis_latch_fault(CHASSIS_FAULT_DRIVE_FEEDBACK_TIMEOUT);
    }
    (void)chassis_take_steer_feedback_snapshot();
    (void)fs_ia10b_maintain(&s_chassis.receiver);
    chassis_update_remote_command();
    now = stm32_time_now_ms();
    now_us = stm32_time_now_us();
    chassis_update_feedback_velocity(now);
    for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        if(s_chassis.steer_feedback_valid[i] &&
           steer_home_feedback_timestamp_is_fresh(
               now, s_chassis.steer_feedback_ms[i],
               CHASSIS_STEER_FEEDBACK_MAX_AGE_MS) &&
           s_chassis.steer_feedback_fault_bits[i] != 0u) {
            chassis_latch_fault(CHASSIS_FAULT_STEER_HARDWARE);
            break;
        }
    }
    if(s_chassis.state.enabled && !chassis_board_safe()) {
        chassis_latch_fault(CHASSIS_FAULT_BOARD_ESTOP);
    }
    if(s_chassis.state.fault_latched) {
        chassis_set_actuators_enabled(false);
        chassis_send_stop_once();
        return CHASSIS_SERVICE_STATUS_FAULT_LATCHED;
    }
    if((uint32_t)(now - s_chassis.last_update_ms) < CHASSIS_UPDATE_PERIOD_MS) {
        return CHASSIS_SERVICE_STATUS_OK;
    }
    s_chassis.last_update_ms = now;
    if(s_chassis.state.source == WR_SOURCE_AUTONOMOUS) {
        const uint64_t age_us = now_us >= s_chassis.velocity_rx_us
                                    ? now_us - s_chassis.velocity_rx_us : UINT64_MAX;
        if(!s_chassis.velocity_seen ||
           age_us >= (uint64_t)s_chassis.velocity_valid_ms * 1000u) {
            s_chassis.autonomous_vx = s_chassis.autonomous_vy =
                s_chassis.autonomous_wz = 0.0f;
            s_chassis.state.command_timed_out = true;
        }
        chassis_set_velocity(s_chassis.autonomous_vx, s_chassis.autonomous_vy,
                             s_chassis.autonomous_wz);
    }
    else if(s_chassis.state.source == WR_SOURCE_MANUAL) {
        s_chassis.state.command_timed_out = false;
        chassis_set_velocity(s_chassis.manual_vx, s_chassis.manual_vy,
                             s_chassis.manual_wz);
    }
    else {
        s_chassis.state.command_timed_out = false;
        chassis_set_velocity(0.0f, 0.0f, 0.0f);
    }
    if(!s_chassis.state.enabled || !chassis_board_safe()) {
        chassis_set_actuators_enabled(false);
        return CHASSIS_SERVICE_STATUS_OK;
    }
    chassis_set_actuators_enabled(true);
    if(s_chassis.state.fault_latched) return CHASSIS_SERVICE_STATUS_FAULT_LATCHED;
    if(s_chassis.steer_startup_state == CHASSIS_STEER_STARTUP_ENABLE) {
        if(!chassis_initialize_steering_step()) {
            (void)chassis_record_send_result(false, CHASSIS_FAULT_STEER_TRANSMIT);
            return CHASSIS_SERVICE_STATUS_DEVICE_ERROR;
        }
        for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i)
            (void)benmo_drive_motor_set_target_rpm(&s_chassis.drive,
                                                   s_chassis.drive_ids[i], 0);
        (void)benmo_drive_motor_update(&s_chassis.drive);
        (void)benmo_drive_motor_request_next_feedback(&s_chassis.drive);
        return CHASSIS_SERVICE_STATUS_OK;
    }
    now = stm32_time_now_ms();
    if(s_chassis.steer_startup_state != CHASSIS_STEER_STARTUP_WAIT_FEEDBACK &&
       chassis_steer_feedback_ready() && !chassis_steer_feedback_safe()) {
        chassis_latch_fault(CHASSIS_FAULT_STEER_ANGLE_LIMIT);
        chassis_send_stop_once();
        return CHASSIS_SERVICE_STATUS_FAULT_LATCHED;
    }
    if(s_chassis.steer_startup_state == CHASSIS_STEER_STARTUP_WAIT_FEEDBACK) {
        if(!chassis_steer_feedback_fresh(now)) {
            if((uint32_t)(now - s_chassis.steer_home_start_ms) >=
               CHASSIS_STEER_HOME_TIMEOUT_MS) {
                chassis_latch_fault(CHASSIS_FAULT_STEER_FEEDBACK_TIMEOUT);
                return CHASSIS_SERVICE_STATUS_FAULT_LATCHED;
            }
        }
        else {
            if(!chassis_steer_feedback_safe()) {
                chassis_latch_fault(CHASSIS_FAULT_STEER_ANGLE_LIMIT);
                return CHASSIS_SERVICE_STATUS_FAULT_LATCHED;
            }
            s_chassis.steer_startup_motor_index = 0u;
            s_chassis.steer_startup_waiting_feedback = false;
            s_chassis.steer_home_start_ms = now;
            s_chassis.steer_startup_state = CHASSIS_STEER_STARTUP_PREPARE_HOME;
        }
    }
    if(s_chassis.steer_startup_state == CHASSIS_STEER_STARTUP_PREPARE_HOME) {
        const int8_t prepare_result = chassis_prepare_next_steering_motor(true);
        if((uint32_t)(now - s_chassis.steer_home_start_ms) >=
           CHASSIS_STEER_HOME_TIMEOUT_MS) {
            chassis_latch_fault(CHASSIS_FAULT_STEER_FEEDBACK_TIMEOUT);
            return CHASSIS_SERVICE_STATUS_FAULT_LATCHED;
        }
        if(prepare_result < 0) {
            (void)chassis_record_send_result(false,
                                             CHASSIS_FAULT_STEER_TRANSMIT);
            return CHASSIS_SERVICE_STATUS_DEVICE_ERROR;
        }
        if(prepare_result > 0) {
            s_chassis.steer_home_start_ms = now;
            s_chassis.steer_startup_state = CHASSIS_STEER_STARTUP_HOME;
        }
    }
    if(s_chassis.steer_startup_state == CHASSIS_STEER_STARTUP_HOME) {
        bool steer_home_send_ok = true;
        bool drive_sends_ok = true;
        const bool steer_home_done =
            chassis_run_steer_home(now, &steer_home_send_ok);
        if(!steer_home_send_ok) {
            if(s_chassis.state.fault_latched) {
                return CHASSIS_SERVICE_STATUS_FAULT_LATCHED;
            }
            (void)chassis_record_send_result(false,
                                             CHASSIS_FAULT_STEER_TRANSMIT);
            return CHASSIS_SERVICE_STATUS_DEVICE_ERROR;
        }
        if(!steer_home_done) {
            for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
                (void)benmo_drive_motor_set_target_rpm(&s_chassis.drive,
                                                       s_chassis.drive_ids[i],
                                                       0);
            }
            if(benmo_drive_motor_update(&s_chassis.drive) != BENMO_DRIVE_STATUS_OK ||
               benmo_drive_motor_request_next_feedback(&s_chassis.drive) !=
                   BENMO_DRIVE_STATUS_OK) {
                drive_sends_ok = false;
            }
            if(!drive_sends_ok) {
                (void)chassis_record_send_result(false,
                                                 CHASSIS_FAULT_DRIVE_TRANSMIT);
            }
            else {
                (void)chassis_record_send_result(true, CHASSIS_FAULT_NONE);
            }
            return drive_sends_ok ? CHASSIS_SERVICE_STATUS_OK
                                  : CHASSIS_SERVICE_STATUS_DEVICE_ERROR;
        }
    }
    if(s_chassis.steer_startup_state == CHASSIS_STEER_STARTUP_SWITCH_PP) {
        int8_t prepare_result;
        if((uint32_t)(now - s_chassis.steer_home_start_ms) >=
           CHASSIS_STEER_HOME_TIMEOUT_MS) {
            chassis_latch_fault(CHASSIS_FAULT_STEER_FEEDBACK_TIMEOUT);
            return CHASSIS_SERVICE_STATUS_FAULT_LATCHED;
        }
        prepare_result = chassis_prepare_next_steering_motor(false);
        if(prepare_result < 0) {
            (void)chassis_record_send_result(false,
                                             CHASSIS_FAULT_STEER_TRANSMIT);
            return CHASSIS_SERVICE_STATUS_DEVICE_ERROR;
        }
        if(prepare_result > 0) {
            chassis_seed_steer_targets_from_feedback();
            s_chassis.steer_startup_state = CHASSIS_STEER_STARTUP_READY;
            (void)chassis_record_send_result(true, CHASSIS_FAULT_NONE);
        }
    }
    if(s_chassis.steer_startup_state != CHASSIS_STEER_STARTUP_READY) {
        for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
            (void)benmo_drive_motor_set_target_rpm(&s_chassis.drive,
                                                   s_chassis.drive_ids[i], 0);
        }
        (void)benmo_drive_motor_update(&s_chassis.drive);
        (void)benmo_drive_motor_request_next_feedback(&s_chassis.drive);
        return CHASSIS_SERVICE_STATUS_OK;
    }
    if(!chassis_steer_feedback_fresh(now)) {
        chassis_latch_fault(CHASSIS_FAULT_STEER_FEEDBACK_TIMEOUT);
        return CHASSIS_SERVICE_STATUS_FAULT_LATCHED;
    }
    if(!chassis_steer_run_status(now)) {
        chassis_latch_fault(CHASSIS_FAULT_STEER_HARDWARE);
        return CHASSIS_SERVICE_STATUS_FAULT_LATCHED;
    }
    {
        const BenmoDriveMotorStatus feedback_status =
            benmo_drive_motor_check_feedback(&s_chassis.drive);
        if(feedback_status == BENMO_DRIVE_STATUS_HARDWARE_FAULT ||
           feedback_status == BENMO_DRIVE_STATUS_FEEDBACK_TIMEOUT) {
            chassis_latch_fault(feedback_status == BENMO_DRIVE_STATUS_HARDWARE_FAULT
                                    ? CHASSIS_FAULT_DRIVE_HARDWARE
                                    : CHASSIS_FAULT_DRIVE_FEEDBACK_TIMEOUT);
            return CHASSIS_SERVICE_STATUS_FAULT_LATCHED;
        }
    }
    s_chassis.state.motor_power_ready =
        chassis_steer_run_status(now) && chassis_drive_feedback_ready(now);
    if(!s_chassis.state.motor_power_ready) {
        for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i)
            (void)benmo_drive_motor_set_target_rpm(&s_chassis.drive,
                                                   s_chassis.drive_ids[i], 0);
        if(benmo_drive_motor_stop_all(&s_chassis.drive) != BENMO_DRIVE_STATUS_OK ||
           benmo_drive_motor_request_next_feedback(&s_chassis.drive) !=
               BENMO_DRIVE_STATUS_OK) {
            (void)chassis_record_send_result(false, CHASSIS_FAULT_DRIVE_TRANSMIT);
            return CHASSIS_SERVICE_STATUS_DEVICE_ERROR;
        }
        return CHASSIS_SERVICE_STATUS_OK;
    }
    if(chassis_task_is_running()) {
        const BenmoDriveMotorStatus feedback_status =
            benmo_drive_motor_check_feedback(&s_chassis.drive);
        if(feedback_status == BENMO_DRIVE_STATUS_HARDWARE_FAULT ||
           feedback_status == BENMO_DRIVE_STATUS_FEEDBACK_TIMEOUT) {
            chassis_latch_fault(feedback_status == BENMO_DRIVE_STATUS_HARDWARE_FAULT
                                    ? CHASSIS_FAULT_DRIVE_HARDWARE
                                    : CHASSIS_FAULT_DRIVE_FEEDBACK_TIMEOUT);
            return CHASSIS_SERVICE_STATUS_FAULT_LATCHED;
        }
        for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
            (void)benmo_drive_motor_set_target_rpm(&s_chassis.drive,
                                                   s_chassis.drive_ids[i], 0);
        }
        if(!chassis_run_task(now_us)) {
            if(s_chassis.state.fault_latched)
                return CHASSIS_SERVICE_STATUS_FAULT_LATCHED;
            (void)chassis_record_send_result(false, CHASSIS_FAULT_STEER_TRANSMIT);
            return CHASSIS_SERVICE_STATUS_DEVICE_ERROR;
        }
        if(benmo_drive_motor_stop_all(&s_chassis.drive) != BENMO_DRIVE_STATUS_OK ||
           benmo_drive_motor_request_next_feedback(&s_chassis.drive) !=
               BENMO_DRIVE_STATUS_OK) {
            (void)chassis_record_send_result(false, CHASSIS_FAULT_STEER_TRANSMIT);
            return CHASSIS_SERVICE_STATUS_DEVICE_ERROR;
        }
        s_chassis.state.update_count++;
        return CHASSIS_SERVICE_STATUS_OK;
    }
    s_chassis.kinematics.model.wheel_radius =
        s_chassis.state.mode == WR_MODE_ON_RAIL ?
            s_chassis.config.rail_radius_m : s_chassis.config.model.wheel_radius;
    if(s_chassis.kinematics.model.wheel_radius <= 0.0f) {
        chassis_latch_fault(CHASSIS_FAULT_KINEMATICS);
        return CHASSIS_SERVICE_STATUS_KINEMATICS_ERROR;
    }
    if(steer_wheel_ik(&s_chassis.kinematics) != STEER_WHEEL_OK) {
        chassis_latch_fault(CHASSIS_FAULT_KINEMATICS);
        return CHASSIS_SERVICE_STATUS_KINEMATICS_ERROR;
    }
    if(s_chassis.config.legacy_steer_57_correction &&
       steer_wheel_apply_legacy_57_correction(&s_chassis.kinematics) !=
           STEER_WHEEL_OK) {
        chassis_latch_fault(CHASSIS_FAULT_KINEMATICS);
        return CHASSIS_SERVICE_STATUS_KINEMATICS_ERROR;
    }
    {
        float feedback_reference[BENMO_DRIVE_MOTOR_COUNT];
        for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
            /* 纯旋转以零位选择成对的舵角/轮速，避免受前次 vy 舵角影响 */
            feedback_reference[i] =
                (s_chassis.kinematics.control.vx == 0.0f &&
                 s_chassis.kinematics.control.vy == 0.0f &&
                 s_chassis.kinematics.control.wz != 0.0f)
                    ? 0.0f
                    : s_chassis.steer_feedback_angle[i];
        }
        if(steer_wheel_optimize_targets(&s_chassis.kinematics,
                                        feedback_reference) !=
           STEER_WHEEL_OK) {
            chassis_latch_fault(CHASSIS_FAULT_KINEMATICS);
            return CHASSIS_SERVICE_STATUS_KINEMATICS_ERROR;
        }
    }

    for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        int16_t rpm;
        float rpm_float = s_chassis.kinematics.control.wheels[i].wheel_omega *
                          CHASSIS_RAD_S_TO_RPM;
        const float raw_target = steer_home_raw_position_target(
            s_chassis.kinematics.control.wheels[i].steer_angle,
            s_chassis.steer_feedback_raw_angle[i]);
        bool steer_send_ok;
        if(!isfinite(raw_target)) {
            chassis_latch_fault(CHASSIS_FAULT_STEER_ANGLE_LIMIT);
            return CHASSIS_SERVICE_STATUS_FAULT_LATCHED;
        }
        /* 电机安装方向在驱动映射中固定处理，反馈使用同一映射 */
        rpm = (int16_t)(chassis_limit_float(rpm_float,
                                           BENMO_DRIVE_MOTOR_RPM_MIN,
                                           BENMO_DRIVE_MOTOR_RPM_MAX) *
                        s_chassis.drive_speed_scale);
        (void)benmo_drive_motor_set_target_rpm(&s_chassis.drive,
                                               s_chassis.drive_ids[i], rpm);
        steer_send_ok =
            rs06_steer_motor_set_raw_position_target(
                &s_chassis.steer, s_chassis.steer_ids[i],
                raw_target) ==
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
    {
      const BenmoDriveMotorStatus feedback_status =
          benmo_drive_motor_check_feedback(&s_chassis.drive);
      if(feedback_status == BENMO_DRIVE_STATUS_HARDWARE_FAULT) {
          chassis_latch_fault(CHASSIS_FAULT_DRIVE_HARDWARE);
          return CHASSIS_SERVICE_STATUS_FAULT_LATCHED;
      }
      if(feedback_status == BENMO_DRIVE_STATUS_FEEDBACK_TIMEOUT) {
        chassis_latch_fault(CHASSIS_FAULT_DRIVE_FEEDBACK_TIMEOUT);
        return CHASSIS_SERVICE_STATUS_FAULT_LATCHED;
      }
    }

    s_chassis.state.update_count++;
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
    s_chassis.state.source = WR_SOURCE_NONE;
    s_chassis.state.enabled = false;
    s_chassis.state.motor_power_ready = false;
    s_chassis.session_id = 0u;
    s_chassis.autonomous_vx = s_chassis.autonomous_vy = s_chassis.autonomous_wz = 0.0f;
    s_chassis.velocity_rx_us = 0u;
    s_chassis.velocity_seen = false;
    chassis_cancel_task(WR_TASK_SAFETY_STOPPED, WR_RESULT_SAFETY_STOP);
    chassis_latch_fault(CHASSIS_FAULT_MANUAL_STOP);
    return CHASSIS_SERVICE_STATUS_OK;
}

ChassisServiceStatus chassis_service_fault_clear(void) {
    uint8_t i;
    if(!s_chassis.state.initialized) {
        return CHASSIS_SERVICE_STATUS_NOT_INITIALIZED;
    }
    if(!s_chassis.state.fault_latched || !chassis_board_safe() ||
       !chassis_remote_is_safe_for_fault_clear()) {
        return CHASSIS_SERVICE_STATUS_UNSAFE_STATE;
    }
    chassis_set_actuators_enabled(false);
    for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        s_chassis.steer_feedback_valid[i] = false;
        if(rs06_steer_motor_stop(&s_chassis.steer, s_chassis.steer_ids[i]) !=
               RS06_STEER_STATUS_OK ||
           rs06_steer_motor_set_mode(&s_chassis.steer, s_chassis.steer_ids[i],
                                     RS06_STEER_MODE_VELOCITY) !=
               RS06_STEER_STATUS_OK ||
           rs06_steer_motor_set_velocity_target(
               &s_chassis.steer, s_chassis.steer_ids[i], 0.0f) !=
               RS06_STEER_STATUS_OK) {
            return CHASSIS_SERVICE_STATUS_DEVICE_ERROR;
        }
    }
    (void)benmo_drive_motor_reset_feedback_monitor(&s_chassis.drive);
    s_chassis.consecutive_send_failures = 0u;
    s_chassis.stop_sent = false;
    s_chassis.fault_clear_armed = false;
    s_chassis.state.fault = CHASSIS_FAULT_NONE;
    s_chassis.state.fault_latched = false;
    s_chassis.state.source = WR_SOURCE_NONE;
    s_chassis.state.enabled = false;
    s_chassis.state.motor_power_ready = false;
    s_chassis.session_id = 0u;
    memset(&s_chassis.task, 0, sizeof(s_chassis.task));
    s_chassis.task.phase = WR_TASK_IDLE;
    s_chassis.task_sequence = 0u;
    s_chassis.steer_startup_state = CHASSIS_STEER_STARTUP_WAIT_FEEDBACK;
    s_chassis.steer_startup_motor_index = 0u;
    s_chassis.steer_startup_command_step = 0u;
    s_chassis.steer_startup_waiting_feedback = false;
    s_chassis.steer_home_stable_cycles = 0u;
    s_chassis.steer_home_start_ms = stm32_time_now_ms();
    chassis_set_velocity(0.0f, 0.0f, 0.0f);
    return CHASSIS_SERVICE_STATUS_OK;
}

void chassis_service_set_board_inputs(const ChassisBoardInputs* inputs) {
    uint32_t critical_state;
    if(inputs == NULL) return;
    critical_state = stm32_critical_enter();
    s_chassis.board_inputs = *inputs;
    if(!inputs->battery_valid || !isfinite(inputs->battery_voltage) ||
       !isfinite(inputs->battery_current)) {
        s_chassis.board_inputs.battery_valid = false;
        s_chassis.board_inputs.battery_voltage = NAN;
        s_chassis.board_inputs.battery_current = NAN;
    }
    stm32_critical_exit(critical_state);
}

WrResult chassis_service_apply_control(const WrControlCommand* command,
                                       uint32_t sequence, uint64_t rx_us) {
    (void)sequence;
    (void)rx_us;
    if(command == NULL || command->session_id == 0u ||
       command->source > WR_SOURCE_AUTONOMOUS || command->mode > WR_MODE_EXIT) {
        return WR_RESULT_BAD_PARAMETER;
    }
    if(!s_chassis.state.initialized) return WR_RESULT_HARDWARE_FAULT;
    if(command->quick_stop) {
        (void)chassis_service_stop();
        return WR_RESULT_SUCCESS;
    }
    if(command->clear_fault) {
        if(command->enable || command->source != WR_SOURCE_NONE)
            return WR_RESULT_BAD_PARAMETER;
        if(!s_chassis.state.fault_latched) return WR_RESULT_SUCCESS;
        if(chassis_service_fault_clear() != CHASSIS_SERVICE_STATUS_OK)
            return WR_RESULT_SAFETY_STOP;
        return WR_RESULT_SUCCESS;
    }
    if(command->source == WR_SOURCE_MANUAL) return WR_RESULT_UNAUTHORIZED;
    if(command->source == WR_SOURCE_NONE || !command->enable) {
        if(command->source == WR_SOURCE_NONE && command->enable)
            return WR_RESULT_BAD_PARAMETER;
        if(s_chassis.state.source == WR_SOURCE_MANUAL)
            return WR_RESULT_BUSY;
        if(s_chassis.state.source == WR_SOURCE_AUTONOMOUS &&
           s_chassis.session_id != command->session_id)
            return WR_RESULT_UNAUTHORIZED;
        s_chassis.state.source = WR_SOURCE_NONE;
        s_chassis.state.enabled = false;
        s_chassis.state.motor_power_ready = false;
        s_chassis.session_id = 0u;
        s_chassis.autonomous_vx = s_chassis.autonomous_vy =
            s_chassis.autonomous_wz = 0.0f;
        s_chassis.velocity_rx_us = 0u;
        s_chassis.velocity_seen = false;
        chassis_set_velocity(0.0f, 0.0f, 0.0f);
        chassis_set_actuators_enabled(false);
        return WR_RESULT_SUCCESS;
    }
    if(command->source != WR_SOURCE_AUTONOMOUS) return WR_RESULT_BAD_PARAMETER;
    if(s_chassis.state.remote_enabled || s_chassis.state.source == WR_SOURCE_MANUAL)
        return WR_RESULT_BUSY;
    if(s_chassis.state.fault_latched || !chassis_board_safe())
        return WR_RESULT_SAFETY_STOP;
    if(command->mode == WR_MODE_ON_RAIL && s_chassis.config.rail_radius_m <= 0.0f)
        return WR_RESULT_BAD_PARAMETER;
    if(s_chassis.state.source == WR_SOURCE_AUTONOMOUS &&
       s_chassis.state.enabled && s_chassis.session_id != command->session_id)
        return WR_RESULT_BUSY;
    if(s_chassis.session_id != command->session_id ||
       s_chassis.state.mode != command->mode) {
        chassis_cancel_task(WR_TASK_CANCELLED, WR_RESULT_UNAUTHORIZED);
        s_chassis.autonomous_vx = s_chassis.autonomous_vy =
            s_chassis.autonomous_wz = 0.0f;
        s_chassis.velocity_rx_us = 0u;
        s_chassis.velocity_seen = false;
        s_chassis.state.command_timed_out = true;
    }
    s_chassis.session_id = command->session_id;
    s_chassis.state.source = WR_SOURCE_AUTONOMOUS;
    s_chassis.state.mode = command->mode;
    s_chassis.state.enabled = true;
    return WR_RESULT_SUCCESS;
}

WrResult chassis_service_apply_velocity(const WrVelocityCommand* command,
                                        uint32_t sequence, uint64_t rx_us) {
    const uint64_t now_us = stm32_time_now_us();
    if(command == NULL || !isfinite(command->vx) || !isfinite(command->vy) ||
       !isfinite(command->wz) || command->valid_for_ms == 0u ||
       command->valid_for_ms > WR_CAN_MAX_VELOCITY_VALID_MS ||
       command->session_id == 0u) return WR_RESULT_BAD_PARAMETER;
    if(!s_chassis.state.initialized) return WR_RESULT_HARDWARE_FAULT;
    if(s_chassis.state.source != WR_SOURCE_AUTONOMOUS ||
       !s_chassis.state.enabled || s_chassis.session_id != command->session_id)
        return WR_RESULT_UNAUTHORIZED;
    if(s_chassis.state.fault_latched || !chassis_board_safe())
        return WR_RESULT_SAFETY_STOP;
    if(s_chassis.state.mode == WR_MODE_ON_RAIL &&
       (command->vy != 0.0f || command->wz != 0.0f))
        return WR_RESULT_MODE_NOT_ALLOWED;
    if(chassis_task_is_running() &&
       (command->vx != 0.0f || command->vy != 0.0f || command->wz != 0.0f))
        return WR_RESULT_BUSY;
    if(rx_us > now_us || now_us - rx_us >=
       (uint64_t)command->valid_for_ms * 1000u) {
        s_chassis.state.command_timed_out = true;
        return WR_RESULT_TIMEOUT;
    }
    s_chassis.autonomous_vx = command->vx;
    s_chassis.autonomous_vy = command->vy;
    s_chassis.autonomous_wz = command->wz;
    s_chassis.velocity_rx_us = rx_us;
    s_chassis.velocity_seen = true;
    s_chassis.velocity_valid_ms = command->valid_for_ms;
    s_chassis.state.command_timed_out = false;
    s_chassis.state.last_command_sequence = sequence;
    return WR_RESULT_SUCCESS;
}

WrResult chassis_service_apply_steering_task(const WrSteeringTaskCommand* command,
                                             uint32_t sequence, uint64_t rx_us) {
    (void)sequence;
    uint8_t i;
    uint8_t mask;
    uint64_t now_us = stm32_time_now_us();
    if(command == NULL || command->session_id == 0u ||
       command->action < WR_ACTION_PROBE || command->action > WR_ACTION_HOLD ||
       (command->module_mask & (uint8_t)~WR_CAN_ALL_MODULES) != 0u ||
       !isfinite(command->max_angle) || !isfinite(command->angular_speed) ||
       !isfinite(command->timeout_s) || command->task_sequence == 0u) {
        return WR_RESULT_BAD_PARAMETER;
    }
    if(!s_chassis.state.initialized) return WR_RESULT_HARDWARE_FAULT;
    if(s_chassis.state.source != WR_SOURCE_AUTONOMOUS ||
       !s_chassis.state.enabled || s_chassis.session_id != command->session_id)
        return WR_RESULT_UNAUTHORIZED;
    if(s_chassis.state.fault_latched || !chassis_board_safe())
        return WR_RESULT_SAFETY_STOP;
    if(s_chassis.steer_startup_state != CHASSIS_STEER_STARTUP_READY ||
       !s_chassis.state.motor_power_ready) return WR_RESULT_BUSY;
    if(s_chassis.state.mode != WR_MODE_ENTRY &&
       s_chassis.state.mode != WR_MODE_EXIT) return WR_RESULT_MODE_NOT_ALLOWED;
    if(s_chassis.task.task_sequence != 0u &&
       command->task_sequence != s_chassis.task.task_sequence) {
        if((uint32_t)(command->task_sequence - s_chassis.task.task_sequence) >=
           0x80000000u) return WR_RESULT_STALE_SEQUENCE;
        if(chassis_task_is_running()) return WR_RESULT_BUSY;
    }
    mask = command->module_mask;
    if(command->action == WR_ACTION_PROBE) {
        if(mask == 0u || command->max_angle <= 0.0f ||
           command->max_angle > RS06_STEER_SAFE_POSITION_RAD ||
           command->angular_speed <= 0.0f ||
           s_chassis.config.probe_max_speed_rad_s <= 0.0f ||
           command->angular_speed > s_chassis.config.probe_max_speed_rad_s ||
           command->timeout_s <= 0.0f ||
           command->timeout_s > CHASSIS_TASK_MAX_TIMEOUT_S ||
           s_chassis.config.probe_hard_torque_nm <= 0.0f ||
           s_chassis.config.probe_max_temperature_c <= 0.0f)
            return WR_RESULT_BAD_PARAMETER;
        for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
            if((mask & (uint8_t)(1u << i)) != 0u &&
               (s_chassis.config.probe_direction[i] == 0 ||
                s_chassis.config.probe_min_angle[i] >=
                    s_chassis.config.probe_max_angle[i] ||
                !s_chassis.steer_feedback_valid[i] ||
                s_chassis.steer_feedback_angle[i] <
                    s_chassis.config.probe_min_angle[i] ||
                s_chassis.steer_feedback_angle[i] >
                    s_chassis.config.probe_max_angle[i] ||
                (s_chassis.config.probe_direction[i] > 0 &&
                 s_chassis.steer_feedback_angle[i] >=
                     s_chassis.config.probe_max_angle[i]) ||
                (s_chassis.config.probe_direction[i] < 0 &&
                 s_chassis.steer_feedback_angle[i] <=
                     s_chassis.config.probe_min_angle[i])))
                return WR_RESULT_BAD_PARAMETER;
        }
        if(command->task_sequence == s_chassis.task.task_sequence &&
           (mask & (uint8_t)~(s_chassis.task.active_mask |
                             s_chassis.task.holding_mask)) == 0u) {
            return WR_RESULT_SUCCESS;
        }
        if(command->task_sequence == s_chassis.task.task_sequence)
            return WR_RESULT_BUSY;
        if(rx_us > now_us || now_us - rx_us >=
           (uint64_t)(command->timeout_s * 1000000.0f))
            return WR_RESULT_TIMEOUT;
        if(command->task_sequence != s_chassis.task.task_sequence) {
            memset(&s_chassis.task, 0, sizeof(s_chassis.task));
            s_chassis.task.task_sequence = command->task_sequence;
            for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i)
                s_chassis.task_park_raw_angle[i] = s_chassis.steer_feedback_raw_angle[i];
        }
        s_chassis.task.action = WR_ACTION_PROBE;
        s_chassis.task.phase = WR_TASK_EXECUTING;
        s_chassis.task.result = WR_RESULT_SUCCESS;
        s_chassis.task_speed_rad_s = command->angular_speed;
        s_chassis.task_max_angle_rad = command->max_angle;
        s_chassis.task_timeout_us =
            (uint64_t)(command->timeout_s * 1000000.0f);
        s_chassis.task_deadline_us = rx_us + s_chassis.task_timeout_us;
        s_chassis.autonomous_vx = s_chassis.autonomous_vy =
            s_chassis.autonomous_wz = 0.0f;
        s_chassis.velocity_rx_us = 0u;
        s_chassis.velocity_seen = false;
        s_chassis.state.command_timed_out = true;
        for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
            const uint8_t bit = (uint8_t)(1u << i);
            if((mask & bit) == 0u || (s_chassis.task.active_mask & bit) != 0u ||
               (s_chassis.task.holding_mask & bit) != 0u) continue;
            s_chassis.task_start_angle[i] = s_chassis.steer_feedback_angle[i];
            if(rs06_steer_motor_prepare_velocity(&s_chassis.steer,
                                                 s_chassis.steer_ids[i]) !=
               RS06_STEER_STATUS_OK) {
                s_chassis.task.active_mask = 0u;
                s_chassis.task.holding_mask = 0u;
                s_chassis.task.phase = WR_TASK_FAULT;
                s_chassis.task.result = WR_RESULT_HARDWARE_FAULT;
                chassis_latch_fault(CHASSIS_FAULT_STEER_TRANSMIT);
                return WR_RESULT_HARDWARE_FAULT;
            }
            s_chassis.task.active_mask |= bit;
        }
    }
    else {
        if(command->task_sequence != s_chassis.task.task_sequence)
            return WR_RESULT_STALE_SEQUENCE;
        if(mask == 0u && command->action != WR_ACTION_CANCEL)
            return WR_RESULT_BAD_PARAMETER;
        if(command->action == WR_ACTION_CANCEL && mask == 0u)
            mask = (uint8_t)(s_chassis.task.active_mask |
                             s_chassis.task.holding_mask);
        if((mask & (uint8_t)~(s_chassis.task.active_mask |
                               s_chassis.task.holding_mask |
                               s_chassis.task.completed_mask)) != 0u)
            return WR_RESULT_BAD_PARAMETER;
        for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
            const uint8_t bit = (uint8_t)(1u << i);
            if((mask & bit) == 0u) continue;
            if(command->action == WR_ACTION_HOLD) {
                if((s_chassis.task.holding_mask & bit) != 0u) continue;
                if((s_chassis.task.active_mask & bit) == 0u)
                    return WR_RESULT_BAD_PARAMETER;
                if(rs06_steer_motor_set_velocity_target(&s_chassis.steer,
                                                         s_chassis.steer_ids[i], 0.0f) !=
                       RS06_STEER_STATUS_OK ||
                   rs06_steer_motor_prepare_pp_raw(
                       &s_chassis.steer, s_chassis.steer_ids[i],
                       s_chassis.steer_feedback_raw_angle[i]) != RS06_STEER_STATUS_OK) {
                    chassis_latch_fault(CHASSIS_FAULT_STEER_TRANSMIT);
                    return WR_RESULT_HARDWARE_FAULT;
                }
                s_chassis.task.active_mask &= (uint8_t)~bit;
                s_chassis.task.holding_mask |= bit;
                s_chassis.task_park_raw_angle[i] = s_chassis.steer_feedback_raw_angle[i];
                s_chassis.task_release_mask &= (uint8_t)~bit;
            }
            else if(command->action == WR_ACTION_RELEASE) {
                if((s_chassis.task_release_mask & bit) != 0u ||
                   (s_chassis.task.completed_mask & bit) != 0u) continue;
                if(rs06_steer_motor_prepare_velocity(&s_chassis.steer,
                                                     s_chassis.steer_ids[i]) !=
                   RS06_STEER_STATUS_OK) {
                    chassis_latch_fault(CHASSIS_FAULT_STEER_TRANSMIT);
                    return WR_RESULT_HARDWARE_FAULT;
                }
                s_chassis.task.holding_mask &= (uint8_t)~bit;
                s_chassis.task.active_mask |= bit;
                s_chassis.task_release_mask |= bit;
            }
            else {
                if(rs06_steer_motor_set_velocity_target(
                       &s_chassis.steer, s_chassis.steer_ids[i], 0.0f) !=
                       RS06_STEER_STATUS_OK ||
                   rs06_steer_motor_prepare_pp_raw(
                       &s_chassis.steer, s_chassis.steer_ids[i],
                       s_chassis.steer_feedback_raw_angle[i]) !=
                       RS06_STEER_STATUS_OK) {
                    chassis_latch_fault(CHASSIS_FAULT_STEER_TRANSMIT);
                    return WR_RESULT_HARDWARE_FAULT;
                }
                s_chassis.task.active_mask &= (uint8_t)~bit;
                s_chassis.task.holding_mask &= (uint8_t)~bit;
                s_chassis.task_release_mask &= (uint8_t)~bit;
                s_chassis.task_park_raw_angle[i] = s_chassis.steer_feedback_raw_angle[i];
            }
        }
        s_chassis.task.action = command->action;
        if(command->action == WR_ACTION_RELEASE) {
            s_chassis.task.phase = s_chassis.task.active_mask != 0u
                                       ? WR_TASK_EXECUTING : WR_TASK_COMPLETED;
            s_chassis.task_deadline_us = now_us +
                s_chassis.task_timeout_us;
        }
        else if(command->action == WR_ACTION_HOLD) {
            s_chassis.task.phase = s_chassis.task.active_mask != 0u
                                       ? WR_TASK_EXECUTING : WR_TASK_HOLDING;
        }
        else if(!chassis_task_is_running()) {
            s_chassis.task.phase = WR_TASK_CANCELLED;
        }
    }
    return WR_RESULT_SUCCESS;
}

ChassisServiceStatus chassis_service_get_snapshot(ChassisServiceSnapshot* out,
                                                  uint64_t now_us) {
    uint8_t i;
    uint32_t now_ms = stm32_time_now_ms();
    if(out == NULL) return CHASSIS_SERVICE_STATUS_INVALID_PARAM;
    if(!s_chassis.state.initialized) return CHASSIS_SERVICE_STATUS_NOT_INITIALIZED;
    memset(out, 0, sizeof(*out));
    out->chassis.sample_time_us = s_chassis.measured_at_us;
    out->chassis.last_command_sequence = s_chassis.state.last_command_sequence;
    out->chassis.source = s_chassis.state.source;
    out->chassis.mode = s_chassis.state.mode;
    out->chassis.enabled = s_chassis.state.enabled;
    out->chassis.physical_estop =
        s_chassis.board_inputs.estop_valid ?
            s_chassis.board_inputs.physical_estop : s_chassis.config.require_estop;
    out->chassis.remote_online = s_chassis.state.remote_online;
    out->chassis.command_timed_out = s_chassis.state.command_timed_out;
    out->chassis.motor_power_ready = s_chassis.state.motor_power_ready &&
                                     s_chassis.actuators_enabled;
    out->chassis.vx = s_chassis.kinematics.state.cur_vx;
    out->chassis.vy = s_chassis.kinematics.state.cur_vy;
    out->chassis.wz = s_chassis.kinematics.state.cur_wz;
    if(s_chassis.measured_at_us == 0u) {
        out->chassis.vx = NAN;
        out->chassis.vy = NAN;
        out->chassis.wz = NAN;
    }
    out->chassis.fault_bits = s_chassis.state.fault_latched
                                  ? (1u << s_chassis.state.fault) : 0u;
    out->chassis.battery_voltage = NAN;
    out->chassis.battery_current = NAN;
    if(s_chassis.board_inputs.battery_valid) {
        out->chassis.battery_voltage = s_chassis.board_inputs.battery_voltage;
        out->chassis.battery_current = s_chassis.board_inputs.battery_current;
    }
    for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        WrModuleState* module = &out->modules[i];
        BenmoDriveMotorFeedback drive_feedback;
        module->module = i;
        module->steering_angle = NAN;
        module->steering_velocity = NAN;
        module->steering_current = NAN;
        module->steering_torque = NAN;
        module->steering_temperature = NAN;
        module->drive_position = NAN;
        module->drive_velocity = NAN;
        module->drive_current = NAN;
        module->drive_torque = NAN;
        module->drive_temperature = NAN;
        /* 故障停机后反馈有效标志会清除，保留最后收到的电机故障码供诊断 */
        module->steering_fault_bits = s_chassis.steer_feedback_fault_bits[i];
        if(s_chassis.steer_feedback_valid[i] &&
           steer_home_feedback_timestamp_is_fresh(
               now_ms, s_chassis.steer_feedback_ms[i],
               CHASSIS_STEER_FEEDBACK_MAX_AGE_MS)) {
            module->steering_online = true;
            module->zero_valid = s_chassis.steer_startup_state ==
                                 CHASSIS_STEER_STARTUP_READY;
            module->torque_valid = true;
            const uint64_t age_us =
                (uint64_t)(uint32_t)(now_ms - s_chassis.steer_feedback_ms[i]) *
                1000u;
            module->sample_time_us = now_us >= age_us ? now_us - age_us : 0u;
            module->steering_angle = s_chassis.steer_feedback_angle[i];
            module->steering_velocity = s_chassis.steer_feedback_velocity[i];
            module->steering_torque = s_chassis.steer_feedback_torque[i];
            module->steering_temperature = s_chassis.steer_feedback_temperature[i];
        }
        if(benmo_drive_motor_get_feedback(&s_chassis.drive,
                                          s_chassis.drive_ids[i],
                                          &drive_feedback) == BENMO_DRIVE_STATUS_OK &&
           drive_feedback.valid &&
           (uint32_t)(now_ms - drive_feedback.last_update_ms) <
               CHASSIS_DRIVE_FEEDBACK_TIMEOUT_MS) {
            module->drive_online = true;
            module->drive_velocity =
                drive_feedback.speed_rpm * CHASSIS_RPM_TO_RAD_S;
            module->drive_fault_bits = drive_feedback.error_code;
            const uint64_t age_us =
                (uint64_t)(uint32_t)(now_ms - drive_feedback.last_update_ms) *
                1000u;
            const uint64_t drive_sample_us =
                now_us >= age_us ? now_us - age_us : 0u;
            if(module->sample_time_us == 0u ||
               drive_sample_us < module->sample_time_us)
                module->sample_time_us = drive_sample_us;
        }
        if((s_chassis.task.safety_stopped_mask & (uint8_t)(1u << i)) != 0u)
            module->steering_fault_bits |= 0x8000u;
        if(module->sample_time_us > now_us) module->sample_time_us = now_us;
    }
    out->task = s_chassis.task;
    out->active_module_mask = (uint8_t)(s_chassis.task.active_mask |
                                         s_chassis.task.holding_mask);
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
        case CHASSIS_FAULT_STEER_FEEDBACK_TIMEOUT:
            return "STEER_FEEDBACK_TIMEOUT";
        case CHASSIS_FAULT_STEER_ANGLE_LIMIT:
            return "STEER_ANGLE_LIMIT";
        case CHASSIS_FAULT_DRIVE_FEEDBACK_TIMEOUT:
            return "DRIVE_FEEDBACK_TIMEOUT";
        case CHASSIS_FAULT_KINEMATICS:
            return "KINEMATICS";
        case CHASSIS_FAULT_MANUAL_STOP:
            return "MANUAL_STOP";
        case CHASSIS_FAULT_STEER_HARDWARE:
            return "STEER_HARDWARE";
        default:
            return "UNKNOWN";
    }
}
