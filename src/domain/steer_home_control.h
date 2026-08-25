#ifndef _steer_home_control_h_
#define _steer_home_control_h_

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 根据当前反馈角计算沿优弧回到零位的速度命令
 */
float steer_home_speed_command(float feedback_angle_rad, float kp,
                               float max_speed_rad_s);

bool steer_home_feedback_is_safe(float feedback_angle_rad);

float steer_home_normalize_feedback(float feedback_angle_rad);

float steer_home_raw_position_target(float logical_target_rad,
                                     float raw_feedback_rad);

bool steer_home_feedback_timestamp_is_fresh(uint32_t now_ms,
                                            uint32_t sample_ms,
                                            uint32_t max_age_ms);

#endif
