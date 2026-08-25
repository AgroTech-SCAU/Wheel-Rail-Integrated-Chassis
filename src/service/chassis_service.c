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
#define CHASSIS_SEND_FAILURE_LIMIT 3u
#define CHASSIS_LOG_PERIOD_MS 500u
#define CHASSIS_RAD_S_TO_RPM 9.5492965855f
#define CHASSIS_STEER_HOME_KP 6.0f
#define CHASSIS_STEER_HOME_MAX_SPEED_RAD_S 1.2f
#define CHASSIS_STEER_HOME_DEADBAND_RAD 0.03f
#define CHASSIS_STEER_HOME_STABLE_CYCLES 8u
#define CHASSIS_STEER_HOME_TIMEOUT_MS 8000u
#define CHASSIS_STEER_FEEDBACK_MAX_AGE_MS 100u
#define CHASSIS_RAD_TO_MDEG 57295.7795131f

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
    CHASSIS_STEER_STARTUP_WAIT_FEEDBACK = 0,
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
    ChassisServiceState state;
    volatile Stm32FdcanFrame pending_drive_frame;
    volatile bool drive_frame_pending;
    uint8_t drive_ids[BENMO_DRIVE_MOTOR_COUNT];
    uint8_t steer_ids[BENMO_DRIVE_MOTOR_COUNT];
    float last_steer_target[BENMO_DRIVE_MOTOR_COUNT];
    volatile float pending_steer_angle[BENMO_DRIVE_MOTOR_COUNT];
    volatile uint32_t pending_steer_feedback_ms[BENMO_DRIVE_MOTOR_COUNT];
    volatile uint32_t pending_steer_feedback_sequence[BENMO_DRIVE_MOTOR_COUNT];
    volatile bool pending_steer_feedback_valid[BENMO_DRIVE_MOTOR_COUNT];
    float steer_feedback_angle[BENMO_DRIVE_MOTOR_COUNT];
    float steer_feedback_raw_angle[BENMO_DRIVE_MOTOR_COUNT];
    uint32_t steer_feedback_ms[BENMO_DRIVE_MOTOR_COUNT];
    uint32_t steer_feedback_sequence[BENMO_DRIVE_MOTOR_COUNT];
    uint32_t steer_home_last_sequence[BENMO_DRIVE_MOTOR_COUNT];
    bool steer_feedback_valid[BENMO_DRIVE_MOTOR_COUNT];
    ChassisSteerStartupState steer_startup_state;
    uint8_t steer_startup_motor_index;
    uint32_t steer_startup_wait_sequence;
    bool steer_startup_waiting_feedback;
    uint8_t steer_home_stable_cycles;
    uint32_t steer_home_start_ms;
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
static void chassis_steer_rx(const Stm32FdcanFrame* frame, void* context);
static float chassis_limit_float(float value, float minimum, float maximum);
static bool chassis_remote_switch_is(uint16_t value, uint16_t target);
static float chassis_remote_channel_to_norm(uint16_t value);
static float chassis_remote_channel_to_speed(uint16_t value, float maximum);
static RemoteSpeedLimit chassis_remote_speed_limit(uint16_t switch_value);
static void chassis_set_velocity(float vx, float vy, float wz);
static void chassis_update_remote_command(void);
static bool chassis_remote_is_safe_for_fault_clear(void);
static bool chassis_steer_id_to_index(uint8_t motor_id, uint8_t* out_index);
static bool chassis_steer_feedback_ready(void);
static bool chassis_steer_feedback_fresh(uint32_t now);
static bool chassis_steer_feedback_safe(void);
static bool chassis_take_steer_feedback_snapshot(void);
static bool chassis_run_steer_home(uint32_t now, bool* steer_sends_ok);
static int8_t chassis_prepare_next_steering_motor(bool velocity_mode);
static void chassis_seed_steer_targets_from_feedback(void);
static void chassis_log_periodic(uint32_t now);
static void chassis_log_steer_feedback(const char* reason);
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
        return s_chassis.steer_startup_motor_index >= BENMO_DRIVE_MOTOR_COUNT
                   ? 1
                   : 0;
    }

    status = velocity_mode
                 ? rs06_steer_motor_prepare_velocity(&s_chassis.steer,
                                                     s_chassis.steer_ids[i])
                 : rs06_steer_motor_prepare_pp_raw(
                       &s_chassis.steer, s_chassis.steer_ids[i],
                       steer_home_raw_position_target(
                           0.0f, s_chassis.steer_feedback_raw_angle[i]));
    if(status != RS06_STEER_STATUS_OK) {
        return -1;
    }
    {
        const uint32_t critical_state = stm32_critical_enter();
        s_chassis.steer_startup_wait_sequence =
            s_chassis.pending_steer_feedback_sequence[i];
        stm32_critical_exit(critical_state);
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

static bool chassis_steer_feedback_safe(void) {
    uint8_t i;
    for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        if(!s_chassis.steer_feedback_valid[i] ||
           !steer_home_feedback_is_safe(s_chassis.steer_feedback_angle[i])) {
            return false;
        }
    }
    return true;
}

static void chassis_log_periodic(uint32_t now) {
    if((uint32_t)(now - s_chassis.last_log_ms) < CHASSIS_LOG_PERIOD_MS) {
        return;
    }
    s_chassis.last_log_ms = now;
    chassis_log_steer_feedback("periodic");
}

static void chassis_log_steer_feedback(const char* reason) {
    uint8_t valid_mask = 0u;
    uint8_t i;
    for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        if(s_chassis.steer_feedback_valid[i]) {
            valid_mask |= (uint8_t)(1u << i);
        }
    }
    if(strcmp(reason, "periodic") == 0) {
        (void)log_info("[转向反馈 mdeg norm/raw] id5=%ld/%ld id6=%ld/%ld id7=%ld/%ld id8=%ld/%ld valid=0x%02X",
                       (long)(s_chassis.steer_feedback_angle[0] *
                              CHASSIS_RAD_TO_MDEG),
                       (long)(s_chassis.steer_feedback_raw_angle[0] *
                              CHASSIS_RAD_TO_MDEG),
                       (long)(s_chassis.steer_feedback_angle[1] *
                              CHASSIS_RAD_TO_MDEG),
                       (long)(s_chassis.steer_feedback_raw_angle[1] *
                              CHASSIS_RAD_TO_MDEG),
                       (long)(s_chassis.steer_feedback_angle[2] *
                              CHASSIS_RAD_TO_MDEG),
                       (long)(s_chassis.steer_feedback_raw_angle[2] *
                              CHASSIS_RAD_TO_MDEG),
                       (long)(s_chassis.steer_feedback_angle[3] *
                              CHASSIS_RAD_TO_MDEG),
                       (long)(s_chassis.steer_feedback_raw_angle[3] *
                              CHASSIS_RAD_TO_MDEG),
                       (unsigned int)valid_mask);
        return;
    }
    (void)log_info("[转向反馈/%s mdeg] norm id5=%ld id6=%ld id7=%ld id8=%ld valid=0x%02X",
                   reason,
                   (long)(s_chassis.steer_feedback_angle[0] *
                          CHASSIS_RAD_TO_MDEG),
                   (long)(s_chassis.steer_feedback_angle[1] *
                          CHASSIS_RAD_TO_MDEG),
                   (long)(s_chassis.steer_feedback_angle[2] *
                          CHASSIS_RAD_TO_MDEG),
                   (long)(s_chassis.steer_feedback_angle[3] *
                          CHASSIS_RAD_TO_MDEG),
                   (unsigned int)valid_mask);
    (void)log_info("[转向反馈/%s mdeg] raw  id5=%ld id6=%ld id7=%ld id8=%ld",
                   reason,
                   (long)(s_chassis.steer_feedback_raw_angle[0] *
                          CHASSIS_RAD_TO_MDEG),
                   (long)(s_chassis.steer_feedback_raw_angle[1] *
                          CHASSIS_RAD_TO_MDEG),
                   (long)(s_chassis.steer_feedback_raw_angle[2] *
                          CHASSIS_RAD_TO_MDEG),
                   (long)(s_chassis.steer_feedback_raw_angle[3] *
                          CHASSIS_RAD_TO_MDEG));
    if(strcmp(reason, "fault") == 0) {
        const uint32_t now = stm32_time_now_ms();
        (void)log_info("[转向反馈/fault] age_ms=%ld,%ld,%ld,%ld seq=%lu,%lu,%lu,%lu",
                       (long)(int32_t)(now - s_chassis.steer_feedback_ms[0]),
                       (long)(int32_t)(now - s_chassis.steer_feedback_ms[1]),
                       (long)(int32_t)(now - s_chassis.steer_feedback_ms[2]),
                       (long)(int32_t)(now - s_chassis.steer_feedback_ms[3]),
                       (unsigned long)s_chassis.steer_feedback_sequence[0],
                       (unsigned long)s_chassis.steer_feedback_sequence[1],
                       (unsigned long)s_chassis.steer_feedback_sequence[2],
                       (unsigned long)s_chassis.steer_feedback_sequence[3]);
    }
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
        if(s_chassis.state.initialized) {
            chassis_log_steer_feedback("fault");
        }
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
        if(rs06_steer_motor_stop(&s_chassis.steer, s_chassis.steer_ids[i]) !=
           RS06_STEER_STATUS_OK) {
            return false;
        }
        stm32_time_delay_ms(5u);
        if(rs06_steer_motor_set_mode(&s_chassis.steer, s_chassis.steer_ids[i],
                                     RS06_STEER_MODE_VELOCITY) !=
           RS06_STEER_STATUS_OK) {
            return false;
        }
        stm32_time_delay_ms(5u);
        if(rs06_steer_motor_set_velocity_target(
               &s_chassis.steer, s_chassis.steer_ids[i], 0.0f) !=
           RS06_STEER_STATUS_OK) {
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

    steer_config.ops = &s_steer_ops;
    steer_config.host_id = RS06_STEER_HOST_ID;
    if(rs06_steer_motor_init(&s_chassis.steer, &steer_config) !=
           RS06_STEER_STATUS_OK ||
       !stm32_fdcan_port_register_rx_callback(STM32_FDCAN_BUS_STEER,
                                              chassis_steer_rx, &s_chassis) ||
       !stm32_fdcan_port_init(STM32_FDCAN_BUS_STEER) ||
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
    s_chassis.steer_startup_state = CHASSIS_STEER_STARTUP_WAIT_FEEDBACK;
    s_chassis.steer_startup_motor_index = 0u;
    s_chassis.steer_startup_waiting_feedback = false;
    s_chassis.steer_home_stable_cycles = 0u;
    s_chassis.steer_home_start_ms = stm32_time_now_ms();
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

    (void)chassis_take_steer_feedback_snapshot();
    now = stm32_time_now_ms();
    if((uint32_t)(now - s_chassis.last_update_ms) < CHASSIS_UPDATE_PERIOD_MS) {
        return CHASSIS_SERVICE_STATUS_OK;
    }
    s_chassis.last_update_ms = now;
    chassis_update_remote_command();
    if(s_chassis.steer_startup_state == CHASSIS_STEER_STARTUP_READY) {
        chassis_log_periodic(now);
    }
    (void)chassis_take_steer_feedback_snapshot();
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
            (void)log_info("steer home done; PP ready");
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
    if(steer_wheel_ik(&s_chassis.kinematics) != STEER_WHEEL_OK) {
        chassis_latch_fault(CHASSIS_FAULT_KINEMATICS);
        return CHASSIS_SERVICE_STATUS_KINEMATICS_ERROR;
    }
    if(steer_wheel_apply_legacy_57_correction(&s_chassis.kinematics) !=
       STEER_WHEEL_OK) {
        chassis_latch_fault(CHASSIS_FAULT_KINEMATICS);
        return CHASSIS_SERVICE_STATUS_KINEMATICS_ERROR;
    }
    {
        float feedback_reference[BENMO_DRIVE_MOTOR_COUNT];
        for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
            feedback_reference[i] = s_chassis.steer_feedback_angle[i];
        }
        if(steer_wheel_optimize_targets(&s_chassis.kinematics,
                                        feedback_reference) !=
           STEER_WHEEL_OK) {
            chassis_latch_fault(CHASSIS_FAULT_KINEMATICS);
            return CHASSIS_SERVICE_STATUS_KINEMATICS_ERROR;
        }
    }

    for(i = 0u; i < BENMO_DRIVE_MOTOR_COUNT; ++i) {
        int16_t rpm = (int16_t)(s_chassis.kinematics.control.wheels[i].wheel_omega *
                                CHASSIS_RAD_S_TO_RPM);
        const float raw_target = steer_home_raw_position_target(
            s_chassis.kinematics.control.wheels[i].steer_angle,
            s_chassis.steer_feedback_raw_angle[i]);
        bool steer_send_ok;
        if(!isfinite(raw_target)) {
            chassis_latch_fault(CHASSIS_FAULT_STEER_ANGLE_LIMIT);
            return CHASSIS_SERVICE_STATUS_FAULT_LATCHED;
        }
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
    if(benmo_drive_motor_check_feedback(&s_chassis.drive) ==
       BENMO_DRIVE_STATUS_FEEDBACK_TIMEOUT) {
        chassis_latch_fault(CHASSIS_FAULT_DRIVE_FEEDBACK_TIMEOUT);
        return CHASSIS_SERVICE_STATUS_FAULT_LATCHED;
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
    s_chassis.steer_startup_state = CHASSIS_STEER_STARTUP_WAIT_FEEDBACK;
    s_chassis.steer_startup_motor_index = 0u;
    s_chassis.steer_startup_waiting_feedback = false;
    s_chassis.steer_home_stable_cycles = 0u;
    s_chassis.steer_home_start_ms = stm32_time_now_ms();
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
        default:
            return "UNKNOWN";
    }
}
