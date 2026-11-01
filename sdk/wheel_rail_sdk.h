#ifndef WHEEL_RAIL_SDK_H
#define WHEEL_RAIL_SDK_H

/**
 * @file wheel_rail_sdk.h
 * @brief 上层应用唯一需要包含的头文件；复用 chassis_service_*，不重复包装。
 * @note 单实例；所有 API 必须在同一主循环/任务中串行调用，不得从 ISR 调用。
 *       平台层自行处理接收中断。初始化前由宿主完成 CubeMX/HAL 外设初始化。
 */
#define WHEEL_RAIL_SDK_VERSION_MAJOR 1
#define WHEEL_RAIL_SDK_VERSION_MINOR 0
#define WHEEL_RAIL_SDK_VERSION_PATCH 0
#define WHEEL_RAIL_SDK_VERSION "1.0.0"

#ifdef __cplusplus
extern "C" {
#endif

#include "chassis_service.h"

#ifdef __cplusplus
}
#endif

#endif
