#include "steer_wheel_kinematics.h"

#include <assert.h>

int main(void) {
    SteerWheel wheel;
    const SteerWheelModel model = {
        .length = 0.725f,
        .width = 0.730f,
        .wheel_radius = 0.0215f,
        .max_wheel_linear_speed = 0.45f,
    };
    const float references[4] = { 3.0f, -3.0f, 3.0f, -3.0f };
    uint8_t i;

    assert(steer_wheel_init(&wheel, model) == STEER_WHEEL_OK);
    for(i = 0u; i < 4u; ++i) {
        wheel.control.wheels[i].wheel_omega = 1.0f;
    }
    wheel.control.wheels[0].steer_angle = -3.0f;
    wheel.control.wheels[1].steer_angle = 3.0f;
    wheel.control.wheels[2].steer_angle = -3.0f;
    wheel.control.wheels[3].steer_angle = 3.0f;

    assert(steer_wheel_optimize_targets(&wheel, references) == STEER_WHEEL_OK);
    for(i = 0u; i < 4u; ++i) {
        assert(wheel.control.wheels[i].steer_angle >= -3.091593f);
        assert(wheel.control.wheels[i].steer_angle <= 3.091593f);
        wheel.control.wheels[i].wheel_omega = 0.0f;
    }
    {
        const float stopped_references[4] = {
            3.14159265f, -3.14159265f, 3.14159265f, -3.14159265f
        };
        assert(steer_wheel_optimize_targets(&wheel, stopped_references) ==
               STEER_WHEEL_OK);
    }
    for(i = 0u; i < 4u; ++i) {
        assert(wheel.control.wheels[i].steer_angle >= -3.091593f);
        assert(wheel.control.wheels[i].steer_angle <= 3.091593f);
    }
    return 0;
}
