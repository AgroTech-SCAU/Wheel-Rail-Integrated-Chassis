/**
 * 接入示例：由宿主调用，不含 main，不参与 SDK 库编译。
 * 先完成 HAL/CubeMX 和板卡所需上电等待，再调用 upper_app_init。
 * 上层收到新指令时调用 upper_app_on_velocity；不能循环重放失联前的指令。
 * 所有函数在同一主循环或任务调用。
 */
#include "upper_app.h"

ChassisServiceStatus upper_app_init(void) {
    ChassisServiceConfig config;
    (void)chassis_service_get_default_config(&config);
    config.control_source = CHASSIS_CONTROL_EXTERNAL;
    config.command_timeout_ms = 200u;
    return chassis_service_init_with_config(&config);
}

ChassisServiceStatus upper_app_poll(ChassisServiceState* state) {
    ChassisServiceStatus status = chassis_service_update();
    if(state != 0) {
        (void)chassis_service_get_state(state);
    }
    return status;
}

ChassisServiceStatus upper_app_on_velocity(float vx, float vy, float wz) {
    /* 未就绪时的非零指令会被 SDK 拒绝，上层应检查返回码。 */
    return chassis_service_set_velocity(vx, vy, wz);
}

ChassisServiceStatus upper_app_stop(void) {
    return chassis_service_stop();
}

ChassisServiceStatus upper_app_clear_fault(void) {
    /* 仅在操作人员/上层明确请求恢复时调用；清故障后还需再次等待 ready。 */
    ChassisServiceStatus status = chassis_service_set_velocity(0.0f, 0.0f, 0.0f);
    return status == CHASSIS_SERVICE_STATUS_OK ? chassis_service_fault_clear()
                                                : status;
}
