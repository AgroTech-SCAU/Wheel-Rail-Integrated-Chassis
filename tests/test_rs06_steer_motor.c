#include "rs06_steer_motor.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

static uint32_t sent_id;
static uint8_t sent_data[8];
static uint32_t sent_ids[8];
static uint8_t send_count;
static uint8_t fail_on_send;
static uint8_t delay_count;
static uint32_t delays[8];

static bool fake_send(uint32_t id, const uint8_t* data, uint8_t len) {
    assert(len == 8u);
    sent_id = id;
    memcpy(sent_data, data, len);
    sent_ids[send_count++] = id;
    return fail_on_send == 0u || send_count != fail_on_send;
}

static void fake_delay(uint32_t delay_ms) {
    delays[delay_count++] = delay_ms;
}

static float decode_float(const uint8_t data[4]) {
    float value;
    memcpy(&value, data, sizeof(value));
    return value;
}

int main(void) {
    const Rs06SteerMotorPortOps ops = {
        .send_extended = fake_send,
        .delay_ms = fake_delay,
    };
    const Rs06SteerMotorConfig config = {
        .ops = &ops,
        .host_id = RS06_STEER_HOST_ID,
    };
    Rs06SteerMotor motor = { 0 };
    Rs06SteerMotorFeedback feedback;
    const uint8_t feedback_data[8] = {
        0xA0u, 0x00u, 0x80u, 0x00u, 0x80u, 0x00u, 0x01u, 0x2Cu,
    };

    assert(rs06_steer_motor_init(&motor, &config) == RS06_STEER_STATUS_OK);

    assert(rs06_steer_motor_set_velocity_target(&motor, 6u, -1.25f) ==
           RS06_STEER_STATUS_OK);
    assert(sent_id == 0x1200FD06u);
    assert(sent_data[0] == 0x0Au && sent_data[1] == 0x70u);
    assert(fabsf(decode_float(&sent_data[4]) - (-1.25f)) < 0.0001f);

    assert(rs06_steer_motor_set_mit_position(&motor, 6u, 0.0f, 1.0f, 2.0f,
                                             3.0f, 4.0f) ==
           RS06_STEER_STATUS_OK);
    assert(sent_id == 0x018E3806u);
    assert(sent_data[0] == 0x7Fu && sent_data[1] == 0xFFu);
    assert(sent_data[2] == 0x82u && sent_data[3] == 0x8Eu);
    assert(sent_data[4] == 0x00u && sent_data[5] == 0x1Au);
    assert(sent_data[6] == 0x07u && sent_data[7] == 0xAEu);

    assert(rs06_steer_motor_parse_feedback(&motor, 0x020006FDu, feedback_data,
                                           8u, &feedback) ==
           RS06_STEER_STATUS_OK);
    assert(feedback.motor_id == 6u);
    assert(fabsf(feedback.angle_rad - 3.142f) < 0.002f);

    assert(rs06_steer_motor_parse_feedback(&motor, 0x020005FDu, feedback_data,
                                           8u, &feedback) ==
           RS06_STEER_STATUS_OK);
    assert(feedback.motor_id == 5u);
    assert(fabsf(feedback.angle_rad - (-3.142f)) < 0.002f);

    assert(rs06_steer_motor_parse_feedback(&motor, 0x030005FDu, feedback_data,
                                           8u, &feedback) ==
           RS06_STEER_STATUS_UNSUPPORTED_FRAME);

    send_count = 0u;
    delay_count = 0u;
    fail_on_send = 0u;
    assert(rs06_steer_motor_prepare_pp(&motor, 6u, 1.5f) ==
           RS06_STEER_STATUS_OK);
    assert(send_count == 5u);
    assert(sent_ids[0] == 0x1200FD06u);
    assert(sent_ids[1] == 0x0400FD06u);
    assert(sent_ids[2] == 0x1200FD06u);
    assert(sent_ids[3] == 0x1200FD06u);
    assert(sent_ids[4] == 0x0300FD06u);
    assert(delay_count == 4u);
    assert(delays[0] == 5u && delays[1] == 5u && delays[2] == 5u &&
           delays[3] == 5u);

    assert(rs06_steer_motor_set_position_target(&motor, 6u, 3.0f) ==
           RS06_STEER_STATUS_OK);
    assert(rs06_steer_motor_set_position_target(&motor, 6u, 3.141593f) ==
           RS06_STEER_STATUS_INVALID_PARAM);

    send_count = 0u;
    delay_count = 0u;
    fail_on_send = 3u;
    assert(rs06_steer_motor_prepare_pp(&motor, 6u, 1.5f) ==
           RS06_STEER_STATUS_PORT_ERROR);
    assert(send_count == 3u);

    send_count = 0u;
    delay_count = 0u;
    fail_on_send = 0u;
    assert(rs06_steer_motor_prepare_velocity(&motor, 6u) ==
           RS06_STEER_STATUS_OK);
    assert(send_count == 4u);
    assert(sent_ids[0] == 0x0400FD06u);
    assert(sent_ids[1] == 0x1200FD06u);
    assert(sent_ids[2] == 0x1200FD06u);
    assert(sent_ids[3] == 0x0300FD06u);
    assert(delay_count == 3u);
    return 0;
}
