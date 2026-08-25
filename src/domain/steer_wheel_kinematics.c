/**
 * @file steer_wheel_kinematics.c
 * @brief 四舵轮底盘运动学实现
 */

#include "steer_wheel_kinematics.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

// ! ========================= 变 量 声 明 ========================= ! //

#define sw steer_wheel_interface

#define SW_PI 3.14159265358979323846f
#define SW_2PI (2.0f * SW_PI)
#define SW_HALF_PI (0.5f * SW_PI)
#define SW_EPS 1e-6f

/**
 * @brief 舵轮运动学入口单例定义表
 */
#define X(name, str) .name = STEER_WHEEL_##name,
const struct SteerWheelInterface steer_wheel_interface = {
    { STEER_WHEEL_STATUS_TABLE },
    .init = steer_wheel_init,
    .fk = steer_wheel_fk,
    .ik = steer_wheel_ik,
    .error_code_to_str = steer_wheel_error_code_to_str
};
#undef X

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void sw_get_wheel_pos(const SteerWheelModel* model, float x[4], float y[4]);
static float sw_wrap_pi(float angle);
static bool sw_model_valid(const SteerWheelModel* model);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化舵轮运动学实例
 * @param steer_wheel 舵轮运动学实例指针
 * @param model 底盘模型参数
 * @return SteelWheelErrorCode 错误码
 */
SteelWheelErrorCode steer_wheel_init(SteerWheel* steer_wheel, SteerWheelModel model) {
    if(steer_wheel == NULL)
        return sw.INVALID_PARAM;
    if(!sw_model_valid(&model))
        return sw.INVALID_MODEL;

    steer_wheel->model = model;

    for(uint8_t i = 0; i < 4; ++i) {
        steer_wheel->control.wheels[i].wheel_omega = 0.0f;
        steer_wheel->control.wheels[i].steer_angle = 0.0f;
        steer_wheel->state.cur_wheels[i].wheel_omega = 0.0f;
        steer_wheel->state.cur_wheels[i].steer_angle = 0.0f;
    }

    steer_wheel->control.vx = 0.0f;
    steer_wheel->control.vy = 0.0f;
    steer_wheel->control.wz = 0.0f;
    steer_wheel->state.cur_vx = 0.0f;
    steer_wheel->state.cur_vy = 0.0f;
    steer_wheel->state.cur_wz = 0.0f;
    steer_wheel->initialized = true;

    return sw.OK;
}

/**
 * @brief 正运动学解算，由四个轮模块反馈解算底盘速度
 * @param steer_wheel 舵轮运动学实例指针
 * @return SteelWheelErrorCode 错误码
 */
SteelWheelErrorCode steer_wheel_fk(SteerWheel* steer_wheel) {
    if(steer_wheel == NULL)
        return sw.INVALID_PARAM;
    if(steer_wheel->initialized == false)
        return sw.NOT_INITIALIZE;
    if(!sw_model_valid(&steer_wheel->model))
        return sw.INVALID_MODEL;

    for(uint8_t i = 0; i < 4; ++i) {
        if(!isfinite(steer_wheel->state.cur_wheels[i].wheel_omega) ||
           !isfinite(steer_wheel->state.cur_wheels[i].steer_angle))
            return sw.INVALID_PARAM;
    }

    float x[4];
    float y[4];
    sw_get_wheel_pos(&steer_wheel->model, x, y);

    float sum_vix = 0.0f;
    float sum_viy = 0.0f;
    float sum_w_numer = 0.0f;
    float sum_w_denom = 0.0f;

    for(uint8_t i = 0; i < 4; ++i) {
        const float wheel_linear_speed = steer_wheel->state.cur_wheels[i].wheel_omega * steer_wheel->model.wheel_radius;
        const float steer_angle = steer_wheel->state.cur_wheels[i].steer_angle;

        const float vix = wheel_linear_speed * cosf(steer_angle);
        const float viy = wheel_linear_speed * sinf(steer_angle);

        sum_vix += vix;
        sum_viy += viy;
        sum_w_numer += (-y[i] * vix + x[i] * viy);
        sum_w_denom += (x[i] * x[i] + y[i] * y[i]);
    }

    steer_wheel->state.cur_vx = sum_vix / (float)4;
    steer_wheel->state.cur_vy = sum_viy / (float)4;
    steer_wheel->state.cur_wz = (sum_w_denom > SW_EPS) ? (sum_w_numer / sum_w_denom) : 0.0f;

    return sw.OK;
}

/**
 * @brief 逆运动学解算，由底盘速度指令解算四个轮模块目标
 * @param steer_wheel 舵轮运动学实例指针
 * @return SteelWheelErrorCode 错误码
 */
SteelWheelErrorCode steer_wheel_ik(SteerWheel* steer_wheel) {
    if(steer_wheel == NULL)
        return sw.INVALID_PARAM;
    if(steer_wheel->initialized == false)
        return sw.NOT_INITIALIZE;
    if(!sw_model_valid(&steer_wheel->model))
        return sw.INVALID_MODEL;

    float x[4];
    float y[4];
    sw_get_wheel_pos(&steer_wheel->model, x, y);

    const float vx = steer_wheel->control.vx;
    const float vy = steer_wheel->control.vy;
    const float wz = steer_wheel->control.wz;

    if(!isfinite(vx) || !isfinite(vy) || !isfinite(wz))
        return sw.INVALID_PARAM;

    float max_abs_linear_speed = 0.0f;

    for(uint8_t i = 0; i < 4; ++i) {
        const float vix = vx - wz * y[i];
        const float wheel_vy = (i == 1u || i == 3u) ? -vy : vy;
        const float viy = wheel_vy + wz * x[i];

        float target_linear_speed = sqrtf(vix * vix + viy * viy);
        float target_steer_angle;

        if(target_linear_speed < SW_EPS) {
            target_linear_speed = 0.0f;
            target_steer_angle = sw_wrap_pi(steer_wheel->state.cur_wheels[i].steer_angle);
        }
        else
            target_steer_angle = atan2f(viy, vix);

        steer_wheel->control.wheels[i].wheel_omega = target_linear_speed / steer_wheel->model.wheel_radius;
        steer_wheel->control.wheels[i].steer_angle = target_steer_angle;

        if(fabsf(target_linear_speed) > max_abs_linear_speed)
            max_abs_linear_speed = fabsf(target_linear_speed);
    }

    if(steer_wheel->model.max_wheel_linear_speed > SW_EPS && max_abs_linear_speed > steer_wheel->model.max_wheel_linear_speed) {
        const float scale = steer_wheel->model.max_wheel_linear_speed / max_abs_linear_speed;

        for(uint8_t i = 0; i < 4; ++i)
            steer_wheel->control.wheels[i].wheel_omega *= scale;
    }

    return sw.OK;
}

/**
 * @brief 保留旧底盘对 ID 5/7 所在轮组的旋转方向和 90 度偏置修正
 * @param steer_wheel 舵轮运动学实例；输入单位为 m/s、rad/s，输出为 rad/s、rad
 */
SteelWheelErrorCode steer_wheel_apply_legacy_57_correction(SteerWheel* steer_wheel) {
    const uint8_t indexes[2] = { 0u, 2u };
    const float half_pi = 1.570796327f;
    float half_length;
    float half_width;
    float inverse_wz;
    uint8_t i;

    if(steer_wheel == NULL)
        return sw.INVALID_PARAM;
    if(steer_wheel->initialized == false)
        return sw.NOT_INITIALIZE;
    if(!sw_model_valid(&steer_wheel->model))
        return sw.INVALID_MODEL;
    if(!isfinite(steer_wheel->control.vx) || !isfinite(steer_wheel->control.vy) ||
       !isfinite(steer_wheel->control.wz))
        return sw.INVALID_PARAM;
    if(fabsf(steer_wheel->control.wz) <= SW_EPS)
        return sw.OK;

    half_length = steer_wheel->model.length * 0.5f;
    half_width = steer_wheel->model.width * 0.5f;
    inverse_wz = -steer_wheel->control.wz;

    for(i = 0u; i < 2u; ++i) {
        const uint8_t index = indexes[i];
        const float x = index == 0u ? half_length : -half_length;
        const float y = index == 0u ? half_width : -half_width;
        const float velocity_x = steer_wheel->control.vx - inverse_wz * y;
        const float velocity_y = steer_wheel->control.vy + inverse_wz * x;
        const float speed = sqrtf(velocity_x * velocity_x + velocity_y * velocity_y);
        float angle;

        steer_wheel->control.wheels[index].wheel_omega =
            speed / steer_wheel->model.wheel_radius;
        angle = speed > SW_EPS
                    ? atan2f(velocity_y, velocity_x) + half_pi
                    : steer_wheel->control.wheels[index].steer_angle + half_pi;
        if(angle > SW_PI)
            angle -= SW_2PI;
        steer_wheel->control.wheels[index].steer_angle = angle;
    }

    return sw.OK;
}

/**
 * @brief 根据参考舵角选择转动距离最短的等效舵轮目标
 * @param steer_wheel 舵轮运动学实例指针
 * @param reference_angles 四个舵轮参考角 单位 rad
 * @return SteelWheelErrorCode 错误码
 */
SteelWheelErrorCode steer_wheel_optimize_targets(
    SteerWheel* steer_wheel, const float reference_angles[4]) {
    uint8_t i;

    if(steer_wheel == NULL || reference_angles == NULL)
        return sw.INVALID_PARAM;
    if(steer_wheel->initialized == false)
        return sw.NOT_INITIALIZE;

    for(i = 0u; i < 4u; ++i) {
        float target_angle;
        float normal_delta;
        float reverse_delta;

        if(!isfinite(reference_angles[i]) ||
           !isfinite(steer_wheel->control.wheels[i].wheel_omega) ||
           !isfinite(steer_wheel->control.wheels[i].steer_angle))
            return sw.INVALID_PARAM;

        if(fabsf(steer_wheel->control.wheels[i].wheel_omega) <= SW_EPS) {
            steer_wheel->control.wheels[i].wheel_omega = 0.0f;
            steer_wheel->control.wheels[i].steer_angle =
                sw_wrap_pi(reference_angles[i]);
            continue;
        }

        target_angle = sw_wrap_pi(steer_wheel->control.wheels[i].steer_angle);
        normal_delta = sw_wrap_pi(target_angle - reference_angles[i]);
        reverse_delta =
            sw_wrap_pi(target_angle + SW_PI - reference_angles[i]);
        if(fabsf(reverse_delta) < fabsf(normal_delta)) {
            target_angle = reference_angles[i] + reverse_delta;
            steer_wheel->control.wheels[i].wheel_omega =
                -steer_wheel->control.wheels[i].wheel_omega;
        }
        else
            target_angle = reference_angles[i] + normal_delta;
        steer_wheel->control.wheels[i].steer_angle = target_angle;
    }

    return sw.OK;
}

/**
 * @brief 舵轮运动学错误码转字符串
 * @param status 错误码
 * @return const char* 错误码字符串
 */
#define X(name, str)         \
    case STEER_WHEEL_##name: \
        return str;
const char* steer_wheel_error_code_to_str(SteelWheelErrorCode status) {
    switch(status) {
        STEER_WHEEL_STATUS_TABLE
        default:
            return "UNKNOWN";
    }
}
#undef X

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 根据底盘模型计算四个轮模块相对底盘中心的位置
 * @param model 底盘模型参数
 * @param x 输出四个轮模块 x 坐标，单位 m，顺序为 FL、FR、RR、RL
 * @param y 输出四个轮模块 y 坐标，单位 m，顺序为 FL、FR、RR、RL
 */
static void sw_get_wheel_pos(const SteerWheelModel* model, float x[4], float y[4]) {
    const float hx = model->length * 0.5f;
    const float hy = model->width * 0.5f;

    /* 约定顺序: FL, FR, RR, RL */
    /* 【核心修改点】：将 Y 轴正方向改为向左 (标准 ROS/数学右手坐标系) */
    x[0] = hx;
    y[0] = hy; // 左前 (FL)：Y轴正半轴
    x[1] = hx;
    y[1] = -hy; // 右前 (FR)：Y轴负半轴
    x[2] = -hx;
    y[2] = -hy; // 右后 (RR)：Y轴负半轴
    x[3] = -hx;
    y[3] = hy; // 左后 (RL)：Y轴正半轴
}

/**
 * @brief 将角度归一化到负 pi 开正 pi 闭区间
 * @param angle 输入角度，单位 rad
 * @return float 归一化后的角度，单位 rad
 */
static float sw_wrap_pi(float angle) {
    while(angle > SW_PI) {
        angle -= SW_2PI;
    }
    while(angle <= -SW_PI) {
        angle += SW_2PI;
    }
    return angle;
}

static bool sw_model_valid(const SteerWheelModel* model) {
    return model != NULL &&
           isfinite(model->length) && isfinite(model->width) &&
           isfinite(model->wheel_radius) && isfinite(model->max_wheel_linear_speed) &&
           model->length > 0.0f && model->width > 0.0f &&
           model->wheel_radius > 0.0f && model->max_wheel_linear_speed >= 0.0f;
}
