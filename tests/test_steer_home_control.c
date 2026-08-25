#include "steer_home_control.h"

#include <assert.h>
#include <math.h>

int main(void) {
    assert(fabsf(steer_home_normalize_feedback(6.28318531f)) < 0.0001f);
    assert(fabsf(steer_home_normalize_feedback(-6.28318531f)) < 0.0001f);
    assert(fabsf(steer_home_normalize_feedback(4.71238898f) + 1.57079633f) <
           0.0001f);
    assert(fabsf(steer_home_normalize_feedback(-4.71238898f) - 1.57079633f) <
           0.0001f);
    assert(fabsf(steer_home_raw_position_target(0.0f, 6.28318531f) -
                 6.28318531f) < 0.0001f);
    assert(fabsf(steer_home_raw_position_target(0.5f, -6.28318531f) +
                 5.78318531f) < 0.0001f);
    assert(!isfinite(steer_home_raw_position_target(0.5f, 12.56f)));
    assert(!isfinite(steer_home_raw_position_target(-0.5f, -12.56f)));
    assert(steer_home_feedback_timestamp_is_fresh(1000u, 1050u, 100u));
    assert(steer_home_feedback_timestamp_is_fresh(1100u, 1050u, 100u));
    assert(!steer_home_feedback_timestamp_is_fresh(1201u, 1050u, 100u));
    assert(steer_home_feedback_is_safe(3.14159265f));
    assert(steer_home_feedback_is_safe(-3.14159265f));
    assert(!steer_home_feedback_is_safe(3.14259265f));
    assert(!steer_home_feedback_is_safe(-3.14259265f));
    assert(fabsf(steer_home_speed_command(2.0f, 6.0f, 1.2f) + 1.2f) <
           0.0001f);
    assert(fabsf(steer_home_speed_command(0.01f, 6.0f, 1.2f) + 0.06f) <
           0.0001f);
    return 0;
}
