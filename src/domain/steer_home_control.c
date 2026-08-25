#include "steer_home_control.h"

#include <math.h>

#define STEER_HOME_PI 3.14159265f
#define STEER_HOME_RAW_POSITION_LIMIT 12.57f

bool steer_home_feedback_is_safe(float feedback_angle_rad) {
    return isfinite(feedback_angle_rad) &&
           fabsf(feedback_angle_rad) <= STEER_HOME_PI;
}

float steer_home_normalize_feedback(float feedback_angle_rad) {
    if(!isfinite(feedback_angle_rad)) {
        return feedback_angle_rad;
    }
    while(feedback_angle_rad > STEER_HOME_PI) {
        feedback_angle_rad -= 2.0f * STEER_HOME_PI;
    }
    while(feedback_angle_rad < -STEER_HOME_PI) {
        feedback_angle_rad += 2.0f * STEER_HOME_PI;
    }
    return feedback_angle_rad;
}

float steer_home_raw_position_target(float logical_target_rad,
                                     float raw_feedback_rad) {
    const float target = logical_target_rad + raw_feedback_rad -
                         steer_home_normalize_feedback(raw_feedback_rad);
    if(!isfinite(target) || fabsf(target) > STEER_HOME_RAW_POSITION_LIMIT) {
        return NAN;
    }
    return target;
}

bool steer_home_feedback_timestamp_is_fresh(uint32_t now_ms,
                                            uint32_t sample_ms,
                                            uint32_t max_age_ms) {
    const int32_t age_ms = (int32_t)(now_ms - sample_ms);
    return age_ms < 0 || (uint32_t)age_ms <= max_age_ms;
}

float steer_home_speed_command(float feedback_angle_rad, float kp,
                               float max_speed_rad_s) {
    float error = -feedback_angle_rad;
    float speed;
    if(!isfinite(feedback_angle_rad) || !isfinite(kp) ||
       !isfinite(max_speed_rad_s) || kp < 0.0f || max_speed_rad_s < 0.0f) {
        return 0.0f;
    }
    speed = kp * error;
    if(speed > max_speed_rad_s) {
        return max_speed_rad_s;
    }
    if(speed < -max_speed_rad_s) {
        return -max_speed_rad_s;
    }
    return speed;
}
