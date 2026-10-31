#ifndef WHEEL_RAIL_UPPER_APP_EXAMPLE_H
#define WHEEL_RAIL_UPPER_APP_EXAMPLE_H

#include "wheel_rail_sdk.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 外部模式初始化；由宿主先完成板级初始化和上电等待。 */
ChassisServiceStatus upper_app_init(void);
/** @brief 每轮调用，返回轮询结果；state 可为空，否则同时获取状态快照。 */
ChassisServiceStatus upper_app_poll(ChassisServiceState* state);
/** @brief 只在收到新的业务速度指令时调用，不循环重放旧缓存。 */
ChassisServiceStatus upper_app_on_velocity(float vx, float vy, float wz);
/** @brief 由明确的停止请求触发，锁存停止故障。 */
ChassisServiceStatus upper_app_stop(void);
/** @brief 由明确的恢复请求触发；成功后仍需轮询，等待再次回零。 */
ChassisServiceStatus upper_app_clear_fault(void);

#ifdef __cplusplus
}
#endif

#endif
