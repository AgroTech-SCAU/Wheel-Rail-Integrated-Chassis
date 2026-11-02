#include "host_link_service.h"

#include "chassis_service.h"
#include "stm32_time_port.h"
#include <math.h>
#include <string.h>

#define RESPONSE_CAPACITY 16u
#define TELEMETRY_SLOTS 8u
#define REPLAY_CAPACITY 16u

typedef struct {
    bool valid;
    uint32_t session;
    WrCanFrame request;
    WrCommandAck ack;
} Replay;
typedef struct {
    bool seen;
    uint32_t sequence, session;
} RxSequence;

static struct {
    bool initialized;
    uint32_t boot_id, sequence, rejected;
    WrCanMessage responses[RESPONSE_CAPACITY];
    unsigned response_head, response_count;
    WrCanMessage telemetry[TELEMETRY_SLOTS];
    bool pending[TELEMETRY_SLOTS];
    unsigned selected;
    bool selected_response, selected_valid;
    RxSequence rx[3];
    Replay replay[REPLAY_CAPACITY];
    unsigned replay_next;
    uint64_t chassis_due, imu_due, task_due;
    WrSteeringTaskState last_task;
    unsigned terminal_remaining;
    WrImuSample imu;
    WrImuOrientation orientation;
    bool has_imu, has_orientation;
} s_link;

static bool newer(uint32_t value, uint32_t previous) {
    uint32_t delta = value - previous;
    return delta != 0u && delta < 0x80000000u;
}

static bool push_response(WrCanMessage message) {
    if(s_link.response_count == RESPONSE_CAPACITY) return false;
    message.sequence = s_link.sequence++;
    s_link.responses[(s_link.response_head + s_link.response_count) % RESPONSE_CAPACITY] = message;
    ++s_link.response_count;
    return true;
}

static void set_telemetry(unsigned slot, WrCanMessage message) {
    message.sequence = s_link.sequence++;
    s_link.telemetry[slot] = message;
    s_link.pending[slot] = true;
}

static void send_ack(uint8_t type, uint32_t sequence, WrResult result) {
    ChassisServiceSnapshot snapshot;
    WrCanMessage reply = {0};
    reply.type = WR_CAN_TYPE_ACK;
    reply.data.ack.acknowledged_sequence = sequence;
    reply.data.ack.acknowledged_type = type;
    reply.data.ack.accepted = result == WR_RESULT_SUCCESS;
    reply.data.ack.result = result;
    if(chassis_service_get_snapshot(&snapshot, stm32_time_now_us()) == CHASSIS_SERVICE_STATUS_OK)
        reply.data.ack.active_module_mask = snapshot.active_module_mask;
    if(!push_response(reply)) ++s_link.rejected;
}

bool host_link_service_init(uint32_t boot_id) {
    memset(&s_link, 0, sizeof(s_link));
    if(boot_id == 0u) return false;
    s_link.boot_id = boot_id;
    s_link.initialized = true;
    return true;
}

WrResult host_link_service_receive(const WrCanFrame* frame, uint64_t rx_us) {
    WrCanMessage message;
    WrResult result;
    uint32_t session;
    unsigned index;
    bool wants_ack, stop, safety_action;
    if(!s_link.initialized || frame == 0) return WR_RESULT_BAD_PARAMETER;
    result = wr_can_decode(frame, &message);
    wants_ack = frame->id == WR_CAN_ID_CONTROL || frame->id == WR_CAN_ID_STEERING_TASK;
    if(result != WR_RESULT_SUCCESS) {
        ++s_link.rejected;
        /* Only a complete protocol header can be correlated by the sender. */
        if(wants_ack && frame->length >= 8u && frame->data[0] == WR_CAN_VERSION &&
           frame->data[1] == (frame->id == WR_CAN_ID_CONTROL ? WR_CAN_TYPE_CONTROL : WR_CAN_TYPE_STEERING_TASK)) {
            uint32_t seq = (uint32_t)frame->data[4] | ((uint32_t)frame->data[5] << 8) |
                           ((uint32_t)frame->data[6] << 16) | ((uint32_t)frame->data[7] << 24);
            send_ack(frame->data[1], seq, result);
        }
        return result;
    }
    if(message.type == WR_CAN_TYPE_TIME_REQUEST) {
        WrCanMessage reply = {0};
        reply.type = WR_CAN_TYPE_TIME_RESPONSE;
        reply.data.time_response.host_send_time_ns = message.data.time_request.host_send_time_ns;
        reply.data.time_response.mcu_receive_time_us = rx_us;
        if(push_response(reply)) return WR_RESULT_SUCCESS;
        ++s_link.rejected;
        return WR_RESULT_BUSY;
    }
    if(message.type < WR_CAN_TYPE_VELOCITY || message.type > WR_CAN_TYPE_CONTROL) {
        ++s_link.rejected;
        return WR_RESULT_BAD_FORMAT;
    }
    index = message.type - WR_CAN_TYPE_VELOCITY;
    session = index == 0u ? message.data.velocity.session_id :
              index == 1u ? message.data.steering_task.session_id : message.data.control.session_id;
    stop = message.type == WR_CAN_TYPE_CONTROL && message.data.control.quick_stop;
    safety_action = stop || (message.type == WR_CAN_TYPE_STEERING_TASK &&
                    message.data.steering_task.action == WR_ACTION_CANCEL) ||
                    (message.type == WR_CAN_TYPE_CONTROL &&
                     message.data.control.source == WR_SOURCE_NONE &&
                     !message.data.control.enable && !message.data.control.clear_fault);
    /* Execute stop/cancel/release even if the ACK queue is saturated. Other acknowledged
     * actions are not executed unless their result can be retained for TX. */
    if(wants_ack && s_link.response_count == RESPONSE_CAPACITY && !safety_action) {
        ++s_link.rejected;
        return WR_RESULT_BUSY;
    }
    for(unsigned i = 0; i < REPLAY_CAPACITY; ++i) {
        const Replay* old = &s_link.replay[i];
        if(!safety_action && old->valid && old->session == session && old->request.id == frame->id &&
           old->request.length == frame->length &&
           memcmp(old->request.data, frame->data, frame->length) == 0) {
            if(wants_ack) {
                WrCanMessage reply = {0}; reply.type = WR_CAN_TYPE_ACK; reply.data.ack = old->ack;
                (void)push_response(reply);
            }
            /* In particular, duplicate velocity never refreshes its deadline. */
            return old->ack.result;
        }
    }
    if(!stop && s_link.rx[index].seen && s_link.rx[index].session == session &&
       !newer(message.sequence, s_link.rx[index].sequence)) {
        result = WR_RESULT_STALE_SEQUENCE;
    } else {
        s_link.rx[index].seen = true;
        s_link.rx[index].sequence = message.sequence;
        s_link.rx[index].session = session;
        if(message.type == WR_CAN_TYPE_CONTROL)
            result = chassis_service_apply_control(&message.data.control, message.sequence, rx_us);
        else if(message.type == WR_CAN_TYPE_STEERING_TASK)
            result = chassis_service_apply_steering_task(&message.data.steering_task, message.sequence, rx_us);
        else
            result = chassis_service_apply_velocity(&message.data.velocity, message.sequence, rx_us);
    }
    if(result != WR_RESULT_SUCCESS) ++s_link.rejected;
    if(wants_ack) send_ack(message.type, message.sequence, result);
    if(result != WR_RESULT_STALE_SEQUENCE) {
        Replay* replay = &s_link.replay[s_link.replay_next++ % REPLAY_CAPACITY];
        ChassisServiceSnapshot snapshot;
        memset(replay, 0, sizeof(*replay));
        replay->valid = true; replay->request = *frame; replay->session = session;
        replay->ack.acknowledged_sequence = message.sequence;
        replay->ack.acknowledged_type = message.type;
        replay->ack.accepted = result == WR_RESULT_SUCCESS;
        replay->ack.result = result;
        if(chassis_service_get_snapshot(&snapshot, stm32_time_now_us()) == CHASSIS_SERVICE_STATUS_OK)
            replay->ack.active_module_mask = snapshot.active_module_mask;
    }
    return result;
}

void host_link_service_submit_imu(const WrImuSample* sample) {
    if(sample != 0) { s_link.imu = *sample; s_link.has_imu = true; }
}
void host_link_service_submit_orientation(const WrImuOrientation* sample) {
    if(sample != 0) { s_link.orientation = *sample; s_link.has_orientation = true; }
}

void host_link_service_tick(uint64_t now_us) {
    ChassisServiceSnapshot snapshot;
    WrCanMessage message = {0};
    if(!s_link.initialized) return;
    if(now_us >= s_link.chassis_due &&
       chassis_service_get_snapshot(&snapshot, now_us) == CHASSIS_SERVICE_STATUS_OK) {
        s_link.chassis_due = now_us + 10000u;
        message.type = WR_CAN_TYPE_CHASSIS;
        message.data.chassis = snapshot.chassis;
        message.data.chassis.boot_id = s_link.boot_id;
        set_telemetry(0u, message);
        for(unsigned i = 0; i < 4u; ++i) {
            message.type = WR_CAN_TYPE_MODULE;
            message.data.module = snapshot.modules[i];
            set_telemetry(i + 1u, message);
        }
        if(memcmp(&snapshot.task, &s_link.last_task, sizeof(snapshot.task)) != 0) {
            s_link.last_task = snapshot.task;
            s_link.terminal_remaining = 3u;
            s_link.task_due = now_us;
        }
        if(now_us >= s_link.task_due && (snapshot.task.phase == WR_TASK_EXECUTING ||
           snapshot.task.phase == WR_TASK_HOLDING || s_link.terminal_remaining > 0u)) {
            message.type = WR_CAN_TYPE_TASK_STATE;
            message.data.task_state = snapshot.task;
            set_telemetry(7u, message);
            s_link.task_due = now_us + 20000u;
        }
    }
    if(now_us >= s_link.imu_due) {
        s_link.imu_due = now_us + 5000u;
        message.type = WR_CAN_TYPE_IMU;
        if(s_link.has_imu) message.data.imu = s_link.imu;
        else {
            memset(&message.data.imu, 0, sizeof(message.data.imu));
            for(unsigned i = 0; i < 3u; ++i) message.data.imu.gyro[i] = message.data.imu.accel[i] = NAN;
            message.data.imu.temperature = NAN;
        }
        if(message.data.imu.sample_time_us > now_us || now_us - message.data.imu.sample_time_us > 100000u)
            message.data.imu.status_bits &= ~0x0fu;
        set_telemetry(5u, message);
        if(s_link.has_orientation && (s_link.orientation.status_bits & 4u) != 0u &&
           s_link.orientation.sample_time_us <= now_us && now_us - s_link.orientation.sample_time_us <= 100000u) {
            message.type = WR_CAN_TYPE_ORIENTATION;
            message.data.orientation = s_link.orientation;
            set_telemetry(6u, message);
        } else s_link.pending[6] = false;
    }
}

bool host_link_service_peek_tx(WrCanFrame* frame, uint64_t send_us) {
    WrCanMessage* message;
    if(frame == 0 || !s_link.initialized) return false;
    s_link.selected_valid = false;
    if(s_link.response_count != 0u) {
        s_link.selected_response = true;
        message = &s_link.responses[s_link.response_head];
        if(message->type == WR_CAN_TYPE_TIME_RESPONSE)
            message->data.time_response.mcu_send_time_us = send_us;
    } else {
        unsigned slot;
        for(slot = 0; slot < TELEMETRY_SLOTS && !s_link.pending[slot]; ++slot) {}
        if(slot == TELEMETRY_SLOTS) return false;
        s_link.selected = slot;
        s_link.selected_response = false;
        message = &s_link.telemetry[slot];
    }
    s_link.selected_valid = wr_can_encode(message, frame) == WR_RESULT_SUCCESS;
    return s_link.selected_valid;
}

void host_link_service_commit_tx(void) {
    if(!s_link.selected_valid) return;
    if(s_link.selected_response) {
        s_link.response_head = (s_link.response_head + 1u) % RESPONSE_CAPACITY;
        --s_link.response_count;
    } else {
        s_link.pending[s_link.selected] = false;
        if(s_link.selected == 7u && s_link.terminal_remaining > 0u) --s_link.terminal_remaining;
    }
    s_link.selected_valid = false;
}

uint32_t host_link_service_rejected_frames(void) { return s_link.rejected; }
