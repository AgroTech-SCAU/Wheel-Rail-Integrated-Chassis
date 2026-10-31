/* 模拟平台，调用真实 service/device/domain；不访问服务层私有上下文。 */
#include "wheel_rail_sdk.h"
#include "stm32_board_port.h"
#include "stm32_fdcan_port.h"
#include "stm32_time_port.h"
#include "stm32_uart_port.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(expr) do { \
    if(!(expr)) { \
        fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); \
        exit(1); \
    } \
} while(0)
#define OK(expr) CHECK((expr) == CHASSIS_SERVICE_STATUS_OK)

static uint32_t clock_ms;
static Stm32FdcanRxCallback rx_callbacks[2];
static void* rx_contexts[2];
static Stm32UartEventCallback uart_rx;
static void* uart_context;
static uint8_t* uart_byte;
static unsigned uart_start_count;
static unsigned steer_stop_count;
static unsigned drive_send_count;
static uint8_t last_speed_frame[8];
static bool fail_init;
static bool fail_send;

uint32_t stm32_time_now_ms(void) { return clock_ms; }
void stm32_time_delay_ms(uint32_t ms) { clock_ms += ms; }
uint32_t stm32_critical_enter(void) { return 0u; }
void stm32_critical_exit(uint32_t state) { (void)state; }
void stm32_board_chassis_power_enable(void) {}
bool stm32_fdcan_port_init(Stm32FdcanBus bus) {
    (void)bus;
    return !fail_init;
}
bool stm32_fdcan_port_register_rx_callback(Stm32FdcanBus bus,
                                           Stm32FdcanRxCallback callback,
                                           void* context) {
    rx_callbacks[bus] = callback;
    rx_contexts[bus] = context;
    return true;
}
bool stm32_fdcan_port_send_standard(Stm32FdcanBus bus, uint32_t id,
                                    const uint8_t* data, uint8_t len,
                                    uint32_t timeout) {
    (void)timeout;
    CHECK(bus == STM32_FDCAN_BUS_DRIVE && len == 8u);
    drive_send_count++;
    if(fail_send) return false;
    if(id == 0x032u) memcpy(last_speed_frame, data, 8u);
    if(id == 0x107u) {
        Stm32FdcanFrame feedback = { 0 };
        feedback.id = 0x200u + data[0];
        feedback.id_type = STM32_FDCAN_ID_STANDARD;
        feedback.len = 8u;
        rx_callbacks[bus](&feedback, rx_contexts[bus]);
    }
    return true;
}
bool stm32_fdcan_port_send_extended(Stm32FdcanBus bus, uint32_t id,
                                    const uint8_t* data, uint8_t len,
                                    uint32_t timeout) {
    (void)data;
    (void)timeout;
    CHECK(bus == STM32_FDCAN_BUS_STEER && len == 8u);
    if((id >> 24u) == 4u) steer_stop_count++;
    return !fail_send;
}
bool stm32_uart5_start_receive(uint8_t* data, uint16_t len) {
    CHECK(len == 1u);
    uart_byte = data;
    uart_start_count++;
    return true;
}
bool stm32_uart5_abort_receive(void) { return true; }
bool stm32_uart5_receive_is_ready(void) { return false; }
bool stm32_uart5_register_callbacks(Stm32UartEventCallback rx,
                                    Stm32UartEventCallback error, void* context) {
    (void)error;
    uart_rx = rx;
    uart_context = context;
    return true;
}
bool stm32_usart1_write(const char* data, uint32_t len) {
    (void)data;
    (void)len;
    return true;
}

static ChassisServiceState state(void) {
    ChassisServiceState result;
    OK(chassis_service_get_state(&result));
    return result;
}

static ChassisServiceStatus step(uint32_t ms) {
    clock_ms += ms;
    /* 每个电机提供新鲜零位反馈；通过真实 RS06 解码进入回零状态机。 */
    for(unsigned i = 5u; i <= 8u; ++i) {
        Stm32FdcanFrame feedback = { 0 };
        feedback.id = (2u << 24u) | (i << 8u) | 0xFDu;
        feedback.id_type = STM32_FDCAN_ID_EXTENDED;
        feedback.len = 8u;
        feedback.data[0] = 0x7Fu;
        feedback.data[1] = 0xFFu;
        rx_callbacks[STM32_FDCAN_BUS_STEER](
            &feedback, rx_contexts[STM32_FDCAN_BUS_STEER]);
    }
    return chassis_service_update();
}

static void wait_ready(void) {
    for(unsigned i = 0u; i < 100u && !state().ready; ++i) OK(step(10u));
    CHECK(state().ready);
}

static void initialize(ChassisControlSource source) {
    ChassisServiceConfig config;
    OK(chassis_service_get_default_config(&config));
    CHECK(config.control_source == CHASSIS_CONTROL_REMOTE);
    CHECK(config.model.length == 0.725f && !config.yaw_bias.enabled);
    config.control_source = source;
    config.command_timeout_ms = 0u; /* 验证兼容默认值 */
    OK(chassis_service_init_with_config(&config));
    CHECK(!state().ready && state().initialized);
    CHECK(state().control_source == source);
}

static void external_test(void) {
    initialize(CHASSIS_CONTROL_EXTERNAL);
    CHECK(uart_start_count == 0u);
    CHECK(chassis_service_set_velocity(0.2f, 0, 0) == CHASSIS_SERVICE_STATUS_UNSAFE_STATE);
    OK(chassis_service_set_velocity(0, 0, 0));
    wait_ready();
    CHECK(state().command_vx == 0.0f);
    unsigned count = drive_send_count;
    OK(chassis_service_update());
    CHECK(drive_send_count == count); /* 内部 10 ms 节拍 */

    OK(chassis_service_set_velocity(0.2f, -0.1f, 0.1f));
    CHECK(chassis_service_set_velocity(NAN, 0, 0) == CHASSIS_SERVICE_STATUS_INVALID_PARAM);
    CHECK(chassis_service_set_velocity(0, INFINITY, 0) == CHASSIS_SERVICE_STATUS_INVALID_PARAM);
    CHECK(chassis_service_set_velocity(21, 0, 0) == CHASSIS_SERVICE_STATUS_INVALID_PARAM);
    OK(step(10u));
    CHECK(state().command_vx == 0.2f && state().command_vy == -0.1f);
    CHECK(state().command_wz == 0.1f && state().command_online);
    CHECK(memcmp(last_speed_frame, (uint8_t[8]){0}, 8u) != 0);
    OK(step(190u));
    CHECK(state().command_vx == 0.0f && !state().command_online);
    for(unsigned i = 0u; i < 5u; ++i) OK(step(10u));
    CHECK(memcmp(last_speed_frame, (uint8_t[8]){0}, 8u) == 0);

    ChassisServiceConfig config;
    OK(chassis_service_get_default_config(&config));
    CHECK(chassis_service_init_with_config(&config) == CHASSIS_SERVICE_STATUS_UNSAFE_STATE);
    CHECK(state().ready); /* 重初始化拒绝不破坏现有状态 */
    OK(chassis_service_set_velocity(0.2f, 0, 0));
    OK(step(10u));
    fail_send = true;
    OK(chassis_service_stop());
    unsigned stops = steer_stop_count;
    CHECK(chassis_service_update() == CHASSIS_SERVICE_STATUS_FAULT_LATCHED);
    CHECK(steer_stop_count > stops); /* 停止发送失败后继续重试 */
    fail_send = false;
    CHECK(chassis_service_update() == CHASSIS_SERVICE_STATUS_FAULT_LATCHED);
    CHECK(memcmp(last_speed_frame, (uint8_t[8]){0}, 8u) == 0);
    CHECK(state().fault == CHASSIS_FAULT_MANUAL_STOP && !state().ready);
    CHECK(chassis_service_set_velocity(0.1f, 0, 0) == CHASSIS_SERVICE_STATUS_FAULT_LATCHED);
    CHECK(chassis_service_fault_clear() == CHASSIS_SERVICE_STATUS_UNSAFE_STATE);
    OK(chassis_service_set_velocity(0, 0, 0));
    clock_ms += 200u;
    CHECK(chassis_service_fault_clear() == CHASSIS_SERVICE_STATUS_UNSAFE_STATE);
    OK(chassis_service_set_velocity(0, 0, 0));
    OK(chassis_service_fault_clear());
    CHECK(!state().ready && !state().command_online);
    wait_ready();
    OK(chassis_service_set_velocity(0.1f, 0, 0));
    OK(step(10u));
    CHECK(state().command_vx == 0.1f);
    fail_send = true;
    CHECK(step(10u) == CHASSIS_SERVICE_STATUS_DEVICE_ERROR);
    CHECK(step(10u) == CHASSIS_SERVICE_STATUS_DEVICE_ERROR);
    CHECK(step(10u) == CHASSIS_SERVICE_STATUS_FAULT_LATCHED);
    CHECK(state().fault == CHASSIS_FAULT_STEER_TRANSMIT);
}

static void remote_frame(bool enabled) {
    uint8_t bytes[32] = { 0x20u, 0x40u };
    uint16_t checksum = 0xFFFFu;
    for(unsigned i = 0u; i < 14u; ++i) {
        uint16_t value = i == 1u ? 2000u : 1500u;
        if(i == 9u) value = enabled ? 1000u : 2000u;
        bytes[2u + 2u * i] = (uint8_t)value;
        bytes[3u + 2u * i] = (uint8_t)(value >> 8u);
    }
    for(unsigned i = 0u; i < 30u; ++i) checksum = (uint16_t)(checksum - bytes[i]);
    bytes[30] = (uint8_t)checksum;
    bytes[31] = (uint8_t)(checksum >> 8u);
    for(unsigned i = 0u; i < 32u; ++i) {
        *uart_byte = bytes[i];
        uart_rx(uart_context);
    }
}

static void remote_test(void) {
    OK(chassis_service_init());
    CHECK(uart_start_count == 1u);
    CHECK(chassis_service_set_velocity(0, 0, 0) == CHASSIS_SERVICE_STATUS_UNSAFE_STATE);
    wait_ready();
    remote_frame(true);
    OK(step(10u));
    CHECK(state().remote_online && state().remote_enabled);
    CHECK(state().command_vx == -10.0f);
    remote_frame(false);
    OK(step(10u));
    CHECK(state().command_vx == 0.0f && !state().remote_enabled);
    OK(chassis_service_stop());
    CHECK(chassis_service_fault_clear() == CHASSIS_SERVICE_STATUS_UNSAFE_STATE);
    remote_frame(false);
    OK(chassis_service_fault_clear());
    wait_ready();
    remote_frame(true);
    OK(step(10u));
    OK(step(200u));
    CHECK(!state().remote_online && state().command_vx == 0.0f);
}

static void wrap_test(void) {
    clock_ms = UINT32_MAX - 550u;
    initialize(CHASSIS_CONTROL_EXTERNAL);
    wait_ready();
    OK(chassis_service_set_velocity(0.1f, 0, 0));
    OK(step(10u));
    CHECK(state().command_vx == 0.1f);
    OK(step(100u));
    CHECK(clock_ms < 1000u);
    CHECK(state().command_online && state().command_vx == 0.1f);
    OK(step(90u));
    CHECK(!state().command_online && state().command_vx == 0.0f);
}

static void init_failure_test(void) {
    CHECK(chassis_service_set_velocity(0, 0, 0) == CHASSIS_SERVICE_STATUS_NOT_INITIALIZED);
    CHECK(chassis_service_update() == CHASSIS_SERVICE_STATUS_NOT_INITIALIZED);
    CHECK(chassis_service_get_state(NULL) == CHASSIS_SERVICE_STATUS_INVALID_PARAM);
    CHECK(chassis_service_get_default_config(NULL) == CHASSIS_SERVICE_STATUS_INVALID_PARAM);
    ChassisServiceConfig config;
    OK(chassis_service_get_default_config(&config));
    config.control_source = (ChassisControlSource)99;
    CHECK(chassis_service_init_with_config(&config) == CHASSIS_SERVICE_STATUS_INVALID_PARAM);
    config.control_source = CHASSIS_CONTROL_EXTERNAL;
    fail_init = true;
    CHECK(chassis_service_init_with_config(&config) == CHASSIS_SERVICE_STATUS_INIT_FAILED);
    CHECK(state().fault == CHASSIS_FAULT_INITIALIZATION && !state().ready);
    CHECK(chassis_service_update() == CHASSIS_SERVICE_STATUS_FAULT_LATCHED);
}

int main(int argc, char** argv) {
    CHECK(argc == 2);
    if(strcmp(argv[1], "external") == 0) external_test();
    else if(strcmp(argv[1], "remote") == 0) remote_test();
    else if(strcmp(argv[1], "timeout_wrap") == 0) wrap_test();
    else if(strcmp(argv[1], "init_failure") == 0) init_failure_test();
    else CHECK(false);
    puts("SDK scenario passed");
    return 0;
}
