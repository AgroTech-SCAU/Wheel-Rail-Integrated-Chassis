# Wheel-Rail SDK 使用与修改说明

基于 STM32H723VGT6 的轮轨复合四舵轮底盘 SDK。本文统一介绍项目能力、硬件配置、上层调用方式、构建接入、具体修改和验证结果。

公开入口：[wheel_rail_sdk.h](sdk/wheel_rail_sdk.h)；上层调用示例：[upper_app.c](sdk/examples/upper_app.c) 和 [upper_app.h](sdk/examples/upper_app.h)；构建配置：[CMakeLists.txt](CMakeLists.txt)；源码打包工具：[package_sdk.py](sdk/package_sdk.py)。项目仅保留本文件作为 SDK 说明。

阅读顺序：

1. [SDK 定位与项目概况](#1-sdk-定位与项目概况)
2. [代码如何组成 SDK](#2-代码如何组成-sdk)
3. [两种调用模式](#3-两种调用模式)
4. [接口、单位和返回值](#4-接口单位和返回值)
5. [停止与恢复流程](#5-停止与恢复流程)
6. [调度与硬件边界](#6-调度与硬件边界)
7. [如何编入上层工程](#7-如何编入上层工程)
8. [具体改了哪些代码，为什么这样改](#8-具体改了哪些代码为什么这样改)
9. [验证方法与边界](#9-验证方法与边界)
10. [生命周期与逐接口调用约定](#10-生命周期与逐接口调用约定)
11. [控制链与原车兼容细节](#11-控制链与原车兼容细节)
12. [宿主主循环与任务接入](#12-宿主主循环与任务接入)
13. [平台移植接口与资源归属](#13-平台移植接口与资源归属)
14. [安装导出与源码交付](#14-安装导出与源码交付)
15. [故障码与接入问题排查](#15-故障码与接入问题排查)

## 1. SDK 定位与项目概况

这是运行在底盘 MCU 固件中的 **C11 源码 SDK**，也可以编译成静态库。上层指同一固件内的主循环、RTOS 应用任务或接收上位机指令的业务模块。PC、ROS 或其他处理器需要由宿主实现通信收包，再在 MCU 内调用本 SDK；本次没有新增串口命令协议、ROS 节点或 PC 动态库。

保留原目录和 `chassis_service_*` 命名，通过一个公开头文件、明确的源文件清单、外部速度入口和构建目标形成 SDK。没有复制一套电机控制实现，也没有对已有服务再包一层等价函数。

默认平台仍为 STM32H723VGT6。电机协议、运动学、最近舵角优化、ID 5/7 修正、轮毂方向补偿、偏航补偿、回零过程、反馈轮询、RPM 限幅和斜坡均沿用原代码。

### 1.1 功能特性

- 四舵轮逆运动学、单圈舵角限制与最近等效舵角选择。
- FS-iA10B iBUS 遥控输入、链路超时检测及 VRB/SWB/SWC 映射。
- 四节点本末驱动电机速度控制、反馈轮询、转速限幅与斜坡。
- 四节点 RS06 舵向电机控制、反馈解码和启动回零。
- 外部底盘速度指令输入、有效期检测及超时目标归零。
- CAN 发送错误、驱动反馈超时、故障锁存、停止重试和显式恢复。
- USART1 分级日志，以及兼容 CubeMX 用户代码区的原应用入口。

### 1.2 原板卡硬件配置

| 模块 | 配置 |
|---|---|
| MCU | STM32H723VGT6 Cortex-M7 |
| 驱动电机 | 本末总线电机 4 台，节点 ID 1–4 |
| 舵向电机 | RS06 4 台，节点 ID 5–8 |
| 驱动总线 | FDCAN1，500 kbit/s |
| 舵向总线 | FDCAN2，1 Mbit/s |
| 遥控器 | FS-iA10B，UART5 iBUS，115200-8-N-1 |
| 日志 | USART1，9600-8-N-1 |

### 1.3 默认控制参数

| 参数 | 数值 |
|---|---:|
| 轴距 | 0.725 m |
| 轮距 | 0.730 m |
| 轮半径 | 0.0215 m |
| 原 IK 单轮线速度上限 | 0.45 m/s |
| 驱动转速范围 | -210 至 210 RPM |
| 驱动斜坡 | 每控制周期改变 5 RPM，默认周期为 10 ms |
| 控制周期 | 10 ms |
| 遥控链路超时 | 200 ms |
| 外部指令超时 | 默认 200 ms，可配置 |
| 驱动反馈超时 | 500 ms |

几何参数和偏航补偿可通过初始化配置传入；其他原控制参数仍保留在原模块中，不为 SDK 打包新增全局参数管理器。

## 2. 代码如何组成 SDK

```text
上层应用（原 entry.c / 用户任务 / 通信业务模块）
                     |
             sdk/wheel_rail_sdk.h
                     |
           service/chassis_service.c
           /          |          \
        device      domain       infra
           \          |          /
               platform/*.c
                     |
             CubeMX / STM32 HAL
```

原六层结构仍为 `app -> service -> device/domain/infra/platform`。`app` 通过公开服务接口调用底盘，`device` 通过已有 PortOps 获取平台能力，`domain` 与 `infra` 不直接依赖 STM32 HAL；`platform` 集中访问 HAL 和 CubeMX 外设句柄。

| 目录/文件 | 责任 | 接入 SDK 时的处理 |
|---|---|---|
| `sdk/wheel_rail_sdk.h` | 统一头文件、版本号、C++ 链接边界 | 上层包含这个头文件 |
| `src/service/` | 装配、10 ms 调度、输入选择、回零与故障 | 编入库 |
| `src/device/` | iBUS、本末、RS06 协议 | 原样编入库 |
| `src/domain/` | 舵轮解算、方向优化、回零计算 | 原样编入库 |
| `src/infra/` | 日志格式化 | 原样编入库 |
| `src/platform/` | FDCAN、UART、Tick、临界区、供电使能 | 默认编入库，依赖宿主 HAL |
| `src/app/entry.c` | 原板卡的应用与标定参数 | 原工程继续用；新宿主自行决定是否用 |
| `Core/Src/main.c` | MCU 启动、时钟、外设初始化、主循环 | 属于宿主，不编入 SDK 库 |
| `wheel_rail.ioc` | 原板卡 CubeMX 配置 | 生成宿主外设代码的参考 |
| `sdk/examples/upper_app.c` | 外部模式接入示例 | 按需加入宿主，不编入库 |
| `sdk/examples/upper_app.h` | 示例函数的 C/C++ 声明 | 仅使用示例模块时包含 |
| `sdk/examples/consumer/` | 安装后的独立 CMake 调用工程 | 验证 find_package 接入 |
| `cmake/WheelRailSDKConfig.cmake.in` | 可重定位的 CMake 包配置模板 | 构建安装时自动生成配置 |
| `sdk/package_sdk.py` | 源码 ZIP 与 SHA-256 清单生成/校验 | 交付时运行，无第三方 Python 依赖 |
| `tests/sdk_test.c` | 模拟板卡的回归测试 | 仅测试构建使用 |

公开配置复用了 `ChassisServiceConfig`、`SteerWheelModel` 等已有类型，所以源码交付时还需要保留 `src/service`、`src/domain` 头文件。统一头文件没有把这些类型重新定义一遍。不同版本更换库时，应用和库应一起重新编译；本次不承诺结构体的二进制 ABI 兼容。

## 3. 两种调用模式

### 3.1 原遥控模式：原工程保持原用法

原 `Core/Src/main.c` 在完成 HAL、时钟以及 GPIO、DMA、FDCAN、UART 和定时器初始化后，仍按以下方式启动应用：

```c
entry_init();
while (1) {
    entry_loop();
}
```

`entry_init()` 完成原板卡等待后初始化运动学、电机总线、遥控接收和日志；`entry_loop()` 调用服务轮询，维护接收并按 10 ms 节拍推进遥控映射、运动学、电机下发、反馈轮询和安全检查。

如果宿主直接调用 SDK，则使用下面的服务接口；这与原 `entry_*` 入口二选一，避免重复初始化：

```c
#include "wheel_rail_sdk.h"

/* 在 CubeMX 外设初始化和板卡上电等待后 */
ChassisServiceStatus status = chassis_service_init();
/* 先检查 status；成功后在主循环持续调用 */
status = chassis_service_update();
```

`chassis_service_init()` 使用默认车型参数，关闭偏航补偿，选择 `CHASSIS_CONTROL_REMOTE`。原 `entry_init()` 继续使用已有的自定义配置：偏航补偿开启，`k_vx=-0.065`、`k_vy=0.0466`、死区 `0.01 m/s`；它还保留原来的 1000 ms 等待。**默认 SDK 配置与原 entry 的标定配置有这一区别。**

遥控模式继续使用 UART5、iBUS 和 VRB/SWB/SWC。外部速度 API 在此模式返回 `UNSAFE_STATE`，避免两个输入源竞争。旧的聚合零初始化/指定成员初始化方式在新增字段未填写时仍选择遥控模式；如果手工逐字段赋值，应先整体清零或调用默认配置接口。

### 3.2 外部模式：由业务模块提交底盘速度

```c
#include "wheel_rail_sdk.h"

ChassisServiceStatus app_start(void) {
    ChassisServiceConfig config;
    chassis_service_get_default_config(&config);
    config.control_source = CHASSIS_CONTROL_EXTERNAL;
    config.command_timeout_ms = 200u;
    return chassis_service_init_with_config(&config);
}

/* 在同一个任务中，先轮询，再根据状态处理新收到的业务指令 */
ChassisServiceStatus app_tick(void) {
    return chassis_service_update();
}

ChassisServiceStatus on_new_command(float vx, float vy, float wz) {
    return chassis_service_set_velocity(vx, vy, wz);
}
```

初始化成功不代表回零完成。持续调用 `update()`，通过 `get_state()` 读取 `ready`。`ready=false` 时外部模式仅接受全零速度，非零速度返回 `UNSAFE_STATE`，不会积压到回零结束后执行。`ready=true` 后由上层发送新的运动指令。

外部模式不初始化 iBUS 接收，不要求遥控器在线，不使用 VRB、SWB、SWC，驱动缩放为 `1.0`。仍沿用底层电机限幅和所有原有方向修正。它不会在指令失联时自动切换为遥控模式。控制来源在初始化时确定，运行中不切换。

外部指令需要持续刷新，例如上层以 20–50 ms 的间隔提交**新收到**的指令，必须短于配置有效期。通信失联后应停止提交；不能每次轮询无条件重放缓存，否则 SDK 看到的仍是新指令。满有效期后，下一个控制周期将目标归零，驱动依旧按原来的每周期 5 RPM 斜坡减速，不新增锁存故障。更快的停止请调用 `stop()`。

SDK 没有单独的 start/enable：外部模式回零就绪、无故障且有有效非零指令后便会运行。

## 4. 接口、单位和返回值

上层包含 `wheel_rail_sdk.h` 后，可直接使用以下接口，无需访问电机实例或平台句柄：

```c
ChassisServiceStatus chassis_service_get_default_config(ChassisServiceConfig* out);
ChassisServiceStatus chassis_service_init(void);
ChassisServiceStatus chassis_service_init_with_config(const ChassisServiceConfig* config);
ChassisServiceStatus chassis_service_update(void);
ChassisServiceStatus chassis_service_set_velocity(float vx, float vy, float wz);
ChassisServiceStatus chassis_service_get_state(ChassisServiceState* out);
ChassisServiceStatus chassis_service_stop(void);
ChassisServiceStatus chassis_service_fault_clear(void);
const char* chassis_service_status_str(ChassisServiceStatus status);
const char* chassis_service_fault_str(ChassisFault fault);
```

| 接口 | 用途与约定 |
|---|---|
| `chassis_service_get_default_config(&cfg)` | 只填充配置，不访问硬件；空指针返回 `INVALID_PARAM` |
| `chassis_service_init()` | 以默认配置初始化 |
| `chassis_service_init_with_config(&cfg)` | 初始化全部依赖，配置内容被复制；局部配置变量可以在调用后释放 |
| `chassis_service_update()` | 维护接收并推进控制状态机；未到 10 ms 时不执行新一轮控制，也可返回 `OK` |
| `chassis_service_set_velocity(vx, vy, wz)` | 只在外部模式接收目标，下一控制周期使用；不直接发送 CAN |
| `chassis_service_get_state(&state)` | 复制只读状态；初始化前也能调用，检查 `initialized` 区分生命周期 |
| `chassis_service_stop()` | 锁存 `MANUAL_STOP`，尝试驱动零速及舵向停止；后续 `update()` 重试失败的停止发送 |
| `chassis_service_fault_clear()` | 满足当前模式的清除条件后清锁存，并重新开始回零 |
| `chassis_service_status_str(status)` | 返回静态状态字符串，无需释放 |
| `chassis_service_fault_str(fault)` | 返回静态故障字符串，无需释放 |

`vx`、`vy` 单位为 m/s，`wz` 为 rad/s；必须是有限数，三轴绝对值均不能超过 20。这是接口输入边界，不代表底盘能达到 20 m/s。几何轮速限幅、旧旋转修正、RPM 限幅和方向补偿按原顺序处理；最终驱动转速受 ±210 RPM 和斜坡限制。不要把本次整合视为对原物理速度精度的重新标定。

原运动学坐标约定为 x 向前、y 向左、z 绕竖直轴，但已有实现包含特定轮组的 y 符号和 ID 5/7 旋转修正。本次直接复用这些逻辑，上层使用原车约定接入，实车方向以既有安装及标定为准。轮组顺序为 FL、FR、RR、RL，驱动 ID 1–4 对应舵向 ID 5–8。

| 配置字段 | 意义及默认值 |
|---|---|
| `model.length` / `width` | 轴距 `0.725 m` / 轮距 `0.730 m`，必须有限且为正 |
| `model.wheel_radius` | 轮半径 `0.0215 m`，必须有限且为正 |
| `model.max_wheel_linear_speed` | 原 IK 单轮速度上限 `0.45 m/s`；有限且非负，0 关闭该限幅 |
| `yaw_bias` | 保留原补偿类型；默认关闭；系数有限、死区非负 |
| `control_source` | `REMOTE=0` 或 `EXTERNAL=1` |
| `command_timeout_ms` | 外部指令有效期，0 采用 200 ms；有效范围 1–2147483647 ms；不修改遥控超时 |

| 返回值 | 上层如何理解 |
|---|---|
| `OK` | 本次调用已受理/完成，不代表物理动作已完成；`update` 的 OK 也不代表已经 ready |
| `INVALID_PARAM` | 空指针、非法配置或非法速度；修正输入后调用 |
| `NOT_INITIALIZED` | 尚未成功初始化 |
| `INIT_FAILED` | 初始化依赖失败；读取 fault 并检查板卡/配置 |
| `DEVICE_ERROR` | 本次发送失败；继续轮询以便重试/推进原故障策略 |
| `KINEMATICS_ERROR` | 原运动学链路异常，并锁存故障 |
| `FAULT_LATCHED` | 故障锁存；保持轮询，等待上层明确恢复请求 |
| `UNSAFE_STATE` | 模式不符、未回零、重复初始化或清故障条件不满足 |

`get_state()` 的 `command_vx/vy/wz` 是最近控制周期采用的目标，既不是实测速度，也不是刚提交但尚未轮询的输入。SDK 当前不输出里程计或完整各轮遥测。`update_count` 沿用原定义，仅在正常运行控制链末尾累加，不是 `update()` 调用次数。

`ready` 表示初始化完成、舵向启动状态到达 READY 且无锁存故障；它不是独立的实时硬件健康诊断。`command_online` 在外部模式表示最新被接受的指令未过期，零指令也算在线；遥控模式反映最近控制周期判断的链路状态。外部模式的 `remote_online/remote_enabled` 保持 false。

## 5. 停止与恢复流程

既有故障处理与新增输入模式的行为如下：

- 上电底盘目标速度为零；启动回零期间舵向电机会按原流程动作。
- 遥控失联或 VRB 关闭时，目标速度归零；外部模式指令过期时，目标速度归零。
- 初始化失败、连续三次控制发送失败、驱动反馈超时或运动学异常，会进入原有锁存故障流程。
- 故障锁存后驱动下发零速，RS06 下发停止命令；发送失败时后续轮询继续尝试。
- 锁存故障不自动恢复，由上层根据当前控制模式显式清除。

外部模式：

1. 上层显式调用 `chassis_service_stop()`。它立即尝试停止发送并锁存故障。
2. 继续调用 `update()`；锁存期间非零外部指令被拒绝。
3. 在明确恢复请求到来后，调用 `set_velocity(0, 0, 0)`。这是故障发生后的新零指令。
4. 在该指令有效期内调用 `fault_clear()`。没有新的零指令或零指令已过期会返回 `UNSAFE_STATE`。
5. 清除成功后旧指令作废，重新回零；等待 `ready=true` 后再发送新运动指令。

```c
/* 仅在用户/业务明确请求恢复时执行，不要放进每轮循环自动清故障 */
ChassisServiceStatus status = chassis_service_set_velocity(0, 0, 0);
if(status == CHASSIS_SERVICE_STATUS_OK) {
    status = chassis_service_fault_clear();
}
```

遥控模式继续采用原条件：VRB 关闭、链路在线，清除逻辑须观察到新的有效遥控帧。首次清除请求通常用于记录帧计数，返回 `UNSAFE_STATE`；下一安全帧到达后再由上层显式重试。

`stop()` 的 `OK` 表示停止故障已锁存、停止动作已尝试，不保证电机已经静止或 CAN 全部发送成功。初始化部分失败时 `initialized=false`，无法使用普通清故障流程。已经初始化了电机实例后禁止重复初始化，避免接收中断仍在使用的上下文被清零；这类初始化失败需排查后复位重启。没有新增 deinit、总线释放或运行中换配置接口。

## 6. 调度与硬件边界

- 单实例，无动态分配。所有公开接口在同一主循环或任务中串行调用；多任务上层应自行通过消息队列/互斥串行化，不能从 ISR 调用。
- 中断入口由平台层负责。FDCAN 帧和 UART 字节仍通过原来的回调和缓冲交给服务层。
- `update()` 应持续轮询，内部按 10 ms 节拍执行控制。原 iBUS 队列只有 64 字节，遥控模式尤其应及时维护，不能长期阻塞主循环。
- 初始化和模式准备保留原有阻塞延时；USART1 日志保留原来的同步写入及 `HAL_MAX_DELAY`。因此它是轮询式 SDK，不能承诺每次调用都非阻塞或严格满足 10 ms 最坏时延。
- 舵向回零使用新鲜度 100 ms 的反馈、8 次稳定采样和原 8 s 阶段超时。驱动反馈超时原值为 500 ms，遥控超时为 200 ms。本次未更改这些参数。
- 原代码的运行态转向反馈检查与启动回零检查并不完全相同；本次未额外补充运行态转向失联策略。测试验证的是既有控制链上的 SDK 接入行为。

默认平台会使用 FDCAN1/2、`hfdcan1/2`、`huart1/5`、`CAN1_EN/CAN2_EN` 和 PC15 供电使能。外部模式跳过 UART5 接收初始化，但默认平台源文件仍引用 `huart5`，使用这套适配器时应保留生成的 UART5 句柄。GPIO 和收发器配置由宿主保证。SDK 不包含时钟、DMA、NVIC、启动文件、链接脚本或 HAL 驱动。

平台层定义了 `HAL_FDCAN_RxFifo0Callback`、`HAL_UART_RxCpltCallback`、`HAL_UART_ErrorCallback`。新宿主若已有同名强定义，需要合并到唯一回调中并保留 SDK 分发，不能重复定义。现有 CAN 总线接收槽为专用单用户注册；本次不改造成共享总线管理器。

## 7. 如何编入上层工程

### 7.1 原 EIDE 工程

原 `.eide/eide.yml` 的 `srcDirs` 已扫描 `src`。本次仅给 Debug/Release 增加 `sdk` 头文件路径；`entry.c` 改为包含统一头文件。原 `main.c`、`entry_init()` / `entry_loop()` 和全部标定参数保持原样。

`sdk/examples` 和 `tests` 不在原 EIDE 的 `srcDirs`，不会混入固件。当前仓库忽略了 `Drivers/`、大部分 `Core/` 和链接脚本，新检出后需要从 `wheel_rail.ioc` 生成/恢复这些内容，才能构建完整固件。

补齐生成文件后，原工程的构建与烧录步骤为：

1. 使用 VS Code 打开 `wheel_rail.code-workspace`，安装 Embedded IDE 扩展。
2. 在 EIDE 中选择 `Debug` 或 `Release` target，使用 ARM GCC 构建固件。
3. 按所选 target 的上传配置，通过 OpenOCD 或 J-Link 烧录目标板。

工程配置位于 `.eide/eide.yml`，硬件配置位于 `wheel_rail.ioc`。

### 7.2 其他 IDE 手工接入

编译以下 11 个 `.c`，不要把示例 `main`、原 `entry.c` 或测试自动加入 SDK：

```text
src/service/chassis_service.c
src/device/benmo_drive_motor.c
src/device/rs06_steer_motor.c
src/device/fs_ia10b.c
src/domain/steer_home_control.c
src/domain/steer_wheel_kinematics.c
src/infra/log.c
src/platform/stm32_board_port.c
src/platform/stm32_fdcan_port.c
src/platform/stm32_uart_port.c
src/platform/stm32_time_port.c
```

头文件路径增加 `sdk`、`src/service`、`src/device`、`src/domain`、`src/infra`、`src/platform`，以及宿主的 `Core/Inc` 和 HAL/CMSIS 路径。使用 C11。ARM GCC 链接数学库 `-lm`，并为库与应用选择一致的 Cortex-M7、Thumb、FPU 和浮点 ABI 选项。宿主完成原 main 中相应的 HAL/外设初始化与上电等待后，再调用 SDK 初始化。

上层 C++ 文件包含 `wheel_rail_sdk.h`，底层 `.c` 仍按 C11 编译；统一头文件处理 `extern "C"` 链接边界。

### 7.3 CMake 上层工程

根目录新增 CMake 静态库目标 `wheel_rail_sdk`，别名 `wheel_rail::sdk`。不编译 `Core/Src/main.c` 和 `src/app/entry.c`。

```cmake
# 在宿主中先创建 CubeMX/HAL 目标，它应传播 include 路径和 MCU 宏定义。
# stm32cubemx 是示例名称，请使用实际目标名；CPU/FPU 编译选项也由宿主管理。
add_subdirectory(cmake/stm32cubemx)
set(WHEEL_RAIL_SDK_HAL_TARGET stm32cubemx CACHE STRING "" FORCE)
add_subdirectory(path/to/Wheel-Rail-Integrated-Chassis wheel_rail)
target_link_libraries(my_firmware PRIVATE wheel_rail::sdk)
```

宿主若没有独立的 HAL 目标，也可以在添加子目录后，用 `target_include_directories(wheel_rail_sdk PRIVATE ...)` 和 `target_compile_definitions(wheel_rail_sdk PRIVATE ...)` 提供生成的头文件及 MCU 宏。最终固件仍需链接 HAL 实现、启动代码和链接脚本。不要在宿主再次编译同一批 SDK `.c`，否则会出现重复符号。

`WHEEL_RAIL_SDK_STM32_PORT=OFF` 只排除这四个 STM32 平台实现，保留服务层对相同 `stm32_*` 端口接口的调用。适合模拟测试或由宿主另行实现这四份头文件的全部函数；它不表示已经完成跨平台移植。默认适配源文件没有改名或抽成新 PortOps，以减少本次修改。

在已有 ARM GCC/CMake/Ninja 环境，可单独编译不含 HAL 的 SDK 核心静态库：

```sh
cmake -S . -B build/sdk-arm -G Ninja \
  -DCMAKE_SYSTEM_NAME=Generic \
  -DCMAKE_C_COMPILER=arm-none-eabi-gcc \
  -DCMAKE_TRY_COMPILE_TARGET_TYPE=STATIC_LIBRARY \
  -DWHEEL_RAIL_SDK_STM32_PORT=OFF \
  "-DCMAKE_C_FLAGS=-mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard"
cmake --build build/sdk-arm
```

以上为 shell 换行示意，PowerShell 可写为一行。产物通常为 `build/sdk-arm/libwheel_rail_sdk.a`。它仍有未解析的端口函数，需要宿主提供；不能当作已包含完整 HAL 的固件直接烧录。

## 8. 具体改了哪些代码，为什么这样改

| 文件 | 修改 | 原因 |
|---|---|---|
| `sdk/wheel_rail_sdk.h`（新增） | 统一 include、版本宏、C++ 链接边界 | 上层入口集中，直接复用已存在的服务接口 |
| `src/service/chassis_service.h` | 新增控制来源、指令有效期、默认配置/速度接口及 ready/指令在线字段 | 使业务模块能配置、下发目标、判断是否可运行 |
| `src/service/chassis_service.c` | 原 init 默认值提取为可复用函数；外部指令缓存和输入选择；超时归零；仅遥控模式启动 iBUS；按来源判定清故障条件；禁止活动实例重初始化 | 防止外部输入被遥控映射覆盖，并明确生命周期与恢复行为 |
| `src/app/entry.c` | 使用统一头文件，并把补偿说明指向根 README | 原应用通过统一入口编译；控制参数不变 |
| `.eide/eide.yml` | 两个 target 各增加一条 `sdk` include 路径 | 兼容原工程 |
| `CMakeLists.txt`（新增） | 静态库目标、可选 STM32 适配和模拟测试 | 上层可用 add_subdirectory 复用，无需手动维护全工程 |
| `sdk/examples/upper_app.c`（新增） | 初始化、轮询、收指令、停止、显式恢复函数 | 展示 MCU 内部业务模块如何调用 |
| `sdk/examples/upper_app.h`（新增） | 声明示例入口并支持 C++ 调用 | 示例可直接作为上层模块编译 |
| `sdk/examples/consumer/CMakeLists.txt`（新增） | 使用安装后的 find_package 目标编译示例 | 检查 SDK 不依赖原仓库的绝对路径 |
| `cmake/WheelRailSDKConfig.cmake.in`（新增） | 提供版本、平台配置标记和导入目标 | 支持静态库安装及工程间复用 |
| `sdk/package_sdk.py`（新增） | 白名单源码打包、内容清单、哈希校验 | 交付范围可查，避免把构建缓存或生成文件混入源码包 |
| `tests/sdk_test.c`（新增） | 模拟端口，运行真实服务/协议/运动学 | 检查新增调用路径和遥控兼容性 |
| `README.md` | 合并原项目介绍与 SDK 接入、接口、构建、修改和验证说明，作为唯一 SDK 文档 | 避免分散维护；原 README 的反馈超时由 200 ms 修正为源码已有的 500 ms |

没有移动或改写 `device`、`domain`、`infra`、`platform` 和 `Core/Src/main.c`。原有状态码和故障枚举值保持不变。新增字段追加在原配置/状态结构体末尾，已有指定成员初始化代码仍可源码兼容。

后续细化集中在交付与使用：CMake 增加 install/export、可选示例编译及构建/安装头文件路径分离；服务头文件增加接口契约注释，示例补齐声明。服务实现及原控制算法未在这轮细化中继续修改。全部说明仍集中在根 README，源码包中也只保留一份 README。

## 9. 验证方法与边界

在有本机 C11 编译器的环境运行模拟测试：

```sh
cmake -S . -B build/sdk-host -G Ninja \
  -DWHEEL_RAIL_SDK_STM32_PORT=OFF -DWHEEL_RAIL_SDK_BUILD_TESTS=ON
cmake --build build/sdk-host
ctest --test-dir build/sdk-host --output-on-failure
```

测试通过公开 API 驱动真实服务/电机协议/运动学，只替换平台函数。四个独立进程分别覆盖：外部模式回零、命令输入、节拍、超时、停止重试、故障恢复及连续发送故障；遥控映射/VRB/遥控失联与清故障；毫秒计数回绕；未初始化和初始化失败。测试不包含 HAL 寄存器、中断时序或实物电机。

SDK 核心 ARM 静态库可独立编译验证；完整 STM32 固件链接和实车运动验证需补齐 CubeMX 生成文件后进行。现有 `log.c` 在 ARM GCC 15 的 `-Wextra` 下有两处枚举下界比较警告，本次保留原日志实现，没有为打包 SDK 修改无关代码。

SDK 整合阶段的实际验证结果：

- ARM GNU 15.2.1：SDK 核心静态库、上层 C 示例均编译通过；以 C++11 编译示例也通过，SDK 引用符号保持 C 链接名称。
- Windows 本机：使用临时 TinyCC 0.9.27 的 i386 编译器，四个模拟场景全部通过。该旧工具链的数学头不兼容，需要在忽略的 `build/` 下临时映射到 Windows CRT 数学函数，同时用 `LOG_USE_C11=0` 关闭未被本测试使用的 VOFA 泛型宏。仓库产品源码没有为这些测试工具兼容问题修改；完整 C11 源码由上述 ARM 编译单独验证。
- `git diff --check` 通过。未执行完整 HAL 固件链接、烧录或实车测试。

交付能力细化后的验证：

| 检查项 | 实际结果 |
|---|---|
| CMake 核心库和可选示例模块 | ARM 交叉编译通过 |
| install/export 与独立 find_package 示例 | 在另一个安装前缀下配置、编译通过 |
| 已安装头文件的 C++11 调用 | 交叉编译通过 |
| 已安装核心库与模拟端口完整符号链接 | ARM ELF 链接通过；使用 nosys 桩，仅验证符号和 ABI，不是可直接烧录的固件 |
| 原遥控/外部模式等四个本机模拟场景 | 全部通过 |
| 源码 ZIP 解压后独立构建 | 核心库和示例模块通过，不使用原源码目录 |
| 内容哈希、重复打包和异常归档检查 | 验证清单一致；相同输入的归档哈希一致；修改内容或追加文件被拒绝 |

链接模拟 ARM ELF 时 nosys 会提示若干系统调用未实现，这是验证程序使用桩函数的结果。这里没有替代真实板卡的启动文件、链接脚本或系统调用接入。

## 10. 生命周期与逐接口调用约定

### 10.1 上层可观察的状态

服务内部有回零状态机，上层通过 `initialized`、`ready`、`fault_latched` 和 `command_online` 判断下一步，不直接修改内部实例。

```text
未初始化 --init 成功--> 等待反馈/回零 --轮询推进成功--> 就绪
    |                       |                        |
    +--初始化失败--> 锁存初始化故障                   |
                            +--故障/stop-------------+
                                      |
                                 锁存运行故障
                                      |
                          满足条件且显式 fault_clear
                                      |
                               重新等待反馈/回零
```

| 观察条件 | 含义 | 上层动作 |
|---|---|---|
| `!initialized && !fault_latched` | 还未成功初始化 | 完成硬件准备，再初始化 |
| `!initialized && fault_latched` | 初始化链路失败 | 读取故障；已启动设备后通常需排查并复位 |
| `initialized && !ready && !fault_latched` | 已装配，正在等待反馈或回零 | 持续 update；外部模式只提交零速度 |
| `ready && !command_online` | 软件已就绪，当前无有效输入 | 保持轮询，等待新指令；外部超时目标在控制周期归零 |
| `ready && command_online` | 软件已就绪，输入有效 | 可以提交新速度；零速度也属于有效输入 |
| `fault_latched` | 已锁存故障，ready 为 false | 持续 update 重试停止；根据明确恢复请求清除 |

没有运行中的模式切换、热重配、多底盘实例和自动重连重初始化。上层不得通过重复调用 init 代替故障处理。

### 10.2 各接口的先决条件与副作用

| 接口 | 调用前要求 | 保存/修改什么 | 典型失败与处理 |
|---|---|---|---|
| `get_default_config(out)` | out 非空 | 仅写调用者配置 | INVALID_PARAM：修正指针；不会影响运行实例 |
| `init()` | 宿主硬件就绪、无活动实例 | 创建默认遥控实例 | 与 init_with_config 相同；不会套用 entry 的偏航标定 |
| `init_with_config(cfg)` | cfg 非空、内容有效 | 复制配置、启动端口与设备、进入回零等待 | 非法补偿/来源/有效期返回 INVALID_PARAM；模型检查失败返回 INIT_FAILED 并锁存 |
| `update()` | 需要正常装配或进行故障停止重试 | 维护接收、更新目标、推进控制链 | DEVICE_ERROR 后继续轮询；KINEMATICS_ERROR/FAULT_LATCHED 后检查 state |
| `set_velocity(vx,vy,wz)` | 外部模式；非零还要求 ready 且无故障 | 替换最新三轴速度、记录当前毫秒时间 | 非法值不改旧缓存，也不延长有效期；回零未完成的非零指令直接拒绝 |
| `stop()` | 至少一个电机实例已启动 | 清有效外部输入、目标归零、锁存、尝试停止 | 尚无电机实例时 NOT_INITIALIZED；已有故障时保留首次原因 |
| `fault_clear()` | 初始化成功且故障锁存，满足来源对应的恢复条件 | 重新设置舵向启动流程、复位驱动反馈监控、作废旧外部输入 | 条件不足 UNSAFE_STATE；恢复发送失败 DEVICE_ERROR，锁存仍保留 |
| `get_state(out)` | out 非空；无需先 init | 复制快照，计算 ready 和输入在线标记 | 初始化前返回 OK 和尚未初始化的状态 |
| 两个 `*_str()` | 可在任意生命周期调用 | 返回静态字符串，无实例变更 | 未知枚举值返回 UNKNOWN |

速度提交的检查顺序为“数值合法 → 已初始化 → 外部模式 → 故障中非零 → 回零中非零”。多个条件同时不满足时，返回先遇到的错误。对业务而言应处理枚举意义，不要依赖其整数顺序来决定恢复。

### 10.3 状态快照每个字段的含义

| 字段 | 类型/单位 | 更新时机与边界 |
|---|---|---|
| `command_vx` / `command_vy` | float，m/s | 最近控制周期采用的输入目标；速度提交成功后不会立即变化 |
| `command_wz` | float，rad/s | 原始目标角速度；不含内部偏航前馈补偿结果 |
| `update_count` | uint32_t | 正常控制链末尾递增；回零和锁存轮询不递增，可自然回绕 |
| `fault` | ChassisFault | 当前首次锁存原因；清除成功后为 NONE |
| `initialized` | bool | 全部初始化成功后为 true；不等同于回零完成 |
| `remote_online` | bool | 遥控模式最近一次链路判定；外部模式为 false |
| `remote_enabled` | bool | 遥控模式 VRB 速度使能判定；外部模式为 false |
| `fault_latched` | bool | 是否需要显式恢复；读取不会清除 |
| `control_source` | ChassisControlSource | 当前初始化时选择的来源 |
| `ready` | bool | initialized、舵向状态为 READY 且无锁存；不承诺反馈实时健康 |
| `command_online` | bool | 外部模式按读取时刻判定最新指令有效期；遥控模式沿用最近链路判定 |

因此在超时边界，可能先读到 `command_online=false`，而 `command_vx` 仍是上一控制周期值；下一次到达控制节拍的 update 才将目标归零。状态结构体可以拷贝保存，但它只代表当时的快照，不会随底层自动更新。

### 10.4 指令有效期举例

以有效期 200 ms、某次提交时刻 `t0` 为例：

| 时刻 | 操作 | 结果 |
|---|---|---|
| t0 | `set_velocity(0.2f,0,0)` 成功 | 保存最新指令，开始有效期 |
| 下一个控制节拍 | update | 使用该速度进行解算 |
| t0+100 ms | 再成功提交一条新指令 | 以本次时刻重新计算 200 ms |
| t0+100 ms | 若提交的是 NaN 或越界值 | 拒绝，不刷新 t0 的有效期 |
| t0+200 ms，期间没有成功刷新 | get_state | 外部 command_online 为 false |
| 首个不早于过期时刻的控制节拍 | update | 将目标置零，沿用原驱动斜坡 |

SDK 使用 uint32_t 毫秒差值处理计数回绕；仍要求上层持续轮询。不要暂停整个控制任务后期待库在后台停止，它没有自建线程、中断定时器或后台看门狗。

## 11. 控制链与原车兼容细节

### 11.1 一条速度指令如何变成电机帧

1. 服务选择来源：遥控映射或外部指令缓存；过期输入在这里变为零目标。
2. 回零状态未 READY 时走启动流程，驱动目标保持零，不执行常规运动输出。
3. 原 IK 计算偏航补偿和各轮目标，并执行其原有单轮线速度缩放。
4. 原 `steer_wheel_apply_legacy_57_correction()` 对 ID 5/7 对应轮组执行既有修正。
5. 原最近等效舵角优化选择舵角/反转轮速组合；纯旋转沿用零位参考策略。
6. 轮角速度乘 `9.5492965855` 转 RPM，继续执行服务层方向补偿、±210 RPM 限幅和来源缩放。
7. 本末驱动每轮控制推进最多 5 RPM，生成四节点组合帧；RS06 发送对应原始位置目标。
8. 轮询驱动反馈、统计发送结果、检查原运行故障条件，并在正常结束时增加 update_count。

这条链在本次 SDK 细化中没有改写。尤其是原 IK 限幅后还有旧轮组修正，因此不能把 `max_wheel_linear_speed` 当作对整条控制链重新设计过的速度保证。

### 11.2 轮组、方向与补偿的具体位置

| 数组下标 | 原模型位置 | 驱动 ID | 舵向 ID | 保留的特定逻辑 |
|---|---|---:|---:|---|
| 0 | FL，x 正、y 正 | 1 | 5 | 原 5/7 修正；原始 wz 为零时服务层驱动反向；RS06 协议方向修正 |
| 1 | FR，x 正、y 负 | 2 | 6 | 原 IK 的 vy 分量取反 |
| 2 | RR，x 负、y 负 | 3 | 7 | 原 5/7 修正；本末编码方向修正；RS06 协议方向修正 |
| 3 | RL，x 负、y 正 | 4 | 8 | 原 IK 的 vy 分量取反；原始 wz 为零时服务层驱动反向；本末编码方向修正 |

运动学内部基础计算使用 `vix=vx-wz*y[i]`，`viy=sigma[i]*vy+wz*x[i]`，其中 `sigma={+1,-1,+1,-1}`；再由 atan2 和模长求角度/速度。它包含原车安装适配，上层不要再按相同 ID 重复叠加一次方向修正。

启用偏航补偿时，解算角速度为 `wz + k_vx*vx_effective + k_vy*vy_effective`；某轴绝对值不大于死区时，其补偿用速度为零。原输入 `command_wz` 不被改写，避免每轮重复累加。旧 5/7 分支仍按原始 wz 判断是否进入。

舵向启动回零是依据反馈归一化角回到逻辑零位，并切换 PP；原始位置目标保留反馈所在圈数对应的偏置。它不是每次启动都写电机机械零点，也不会自动执行永久参数保存。

### 11.3 遥控映射保持什么行为

| 遥控通道（从 1 开始） | 内部下标 | 作用 |
|---|---:|---|
| CH1 | 0 | 右摇杆 X → vy，沿用负号映射 |
| CH2 | 1 | 右摇杆 Y → vx，沿用负号映射 |
| CH4 | 3 | 左摇杆 X → wz，沿用负号映射 |
| CH6 / SWB | 5 | 三轴速度档位；接近 2000/1500/1000 时为 20/10/5 |
| CH7 / SWC | 6 | 驱动 RPM 缩放；接近 1000/1500/2000 时为 0.25/0.5/1.0 |
| CH10 / VRB | 9 | 值不大于 1200 时允许速度输入；更高时零目标 |

摇杆中心 1500，归一化跨度 500，中心死区 30；开关比较容差为 250。遥控有效帧按原 iBUS 校验和判定，200 ms 无有效帧则离线。上表中速度档位是映射输入上限，并非最终物理车速。

## 12. 宿主主循环与任务接入

### 12.1 外部模式的完整业务顺序

下面展示宿主组织方式。`host_*` 是由宿主提供的业务事件入口，不是本 SDK 的通信协议；`upper_app_*` 的可编译实现已经包含在示例文件中。

```c
#include "upper_app.h"

/* 宿主实现：返回值 true 只表示本轮存在一个新的对应事件。 */
extern bool host_take_stop_request(void);
extern bool host_take_clear_request(void);
extern bool host_take_new_velocity(float* vx, float* vy, float* wz);
extern void host_report_status(ChassisServiceStatus result,
                               const ChassisServiceState* state);

ChassisServiceStatus firmware_app_init(void) {
    /* HAL、外设初始化及板卡所需等待在调用本函数之前完成。 */
    return upper_app_init();
}

void firmware_app_loop(void) {
    ChassisServiceState state;
    ChassisServiceStatus result = upper_app_poll(&state);
    host_report_status(result, &state);

    if (host_take_stop_request()) {
        result = upper_app_stop();
    } else if (host_take_clear_request()) {
        result = upper_app_clear_fault();
    } else {
        float vx, vy, wz;
        if (!host_take_new_velocity(&vx, &vy, &wz)) {
            return; /* 无新指令也已完成本轮 update，不重放旧缓存。 */
        }
        result = upper_app_on_velocity(vx, vy, wz);
    }
    chassis_service_get_state(&state);
    host_report_status(result, &state);
}
```

main 在 `firmware_app_init()` 成功后持续调用 `firmware_app_loop()`，失败时由宿主处理错误并维持适当轮询。调用示例与原 `entry_init/entry_loop` 二选一。业务收包应丢弃已经过期或回零期间积压的运动事件；SDK 的时间戳从 set_velocity 成功时开始，不知道报文在上游队列中等待了多久。

`host_report_status()` 在上例只用于展示返回码应如何交给业务层；实际实现应避免每轮阻塞输出大量日志。收到 `UNSAFE_STATE` 时保留就绪等待，收到 `FAULT_LATCHED` 时等待明确恢复，不能循环自动调用 clear。

### 12.2 RTOS 接入

把上述初始化、update、set_velocity、stop、fault_clear 和 get_state 全部放在一个底盘任务中执行。其他任务只投递业务消息；UART/CAN ISR 仍走平台接收回调。消息应携带新鲜度信息，由业务任务先过滤积压命令再调用 SDK。

任务调度频率要兼顾原接收队列和阻塞日志，而不是仅依赖“内部周期 10 ms”就直接每 10 ms 维护一次接收。SDK 不调用 RTOS API；宿主自行配置任务优先级、栈大小和等待方式。现有初始化、模式切换和日志可能阻塞，不能从软件定时器回调或高优先级 ISR 直接执行。

### 12.3 C++ 上层

固件底层仍按 C11 构建。C++ 业务文件直接包含 `wheel_rail_sdk.h`；如果使用示例模块，则包含 `upper_app.h`。两者都提供 C 链接边界，不需要把底层 `.c` 改为 `.cpp`。API 不抛出异常，返回值和 bool 状态均由调用者判断。

## 13. 平台移植接口与资源归属

### 13.1 移植范围

保留当前 STM32 适配时，宿主提供 HAL、CubeMX 句柄和 IRQ 分发。迁移到其他板卡或操作系统时，将 `WHEEL_RAIL_SDK_STM32_PORT=OFF`，自行实现下表所有接口；名称继续保持 `stm32_*`，以避免改动服务与协议层。已编译的核心库通常仍会引用遥控相关端口符号，即使运行时选择外部模式，也应提供这些函数的实现或明确的失败桩。

| 端口头文件 | 函数 | 需要满足的约定 |
|---|---|---|
| `stm32_time_port.h` | `stm32_time_now_ms()` | 返回持续运行的 uint32_t 毫秒计数，允许自然回绕；主循环和接收回调用同一时基 |
| 同上 | `stm32_time_delay_ms(ms)` | 保留阻塞等待含义；等待期间应让接收中断可运行 |
| 同上 | `stm32_critical_enter/exit` | 返回并恢复进入前的保护状态，支持嵌套恢复；保护共享反馈和接收状态 |
| `stm32_board_port.h` | `stm32_board_chassis_power_enable()` | 使能本板执行器供电；原实现为 PC15 置高 |
| `stm32_fdcan_port.h` | `*_register_rx_callback(bus,cb,ctx)` | 每条总线保存回调与上下文，禁止静默覆盖其他用户 |
| 同上 | `*_init(bus)` | 配置接收、通知并启动该 CAN 角色；重复调用应兼容原发送路径 |
| 同上 | `*_send_standard/extended(...)` | 发送经典 CAN、最多 8 字节；返回 true 表示复制/入队成功，不能保存短生命周期数据指针等待以后读取 |
| `stm32_uart_port.h` | `stm32_uart5_register_callbacks(...)` | 保存接收完成/错误回调及上下文 |
| 同上 | `stm32_uart5_start_receive(data,len)` | 异步接收至指定缓冲；服务当前每次请求 1 字节，完成后回调 |
| 同上 | `stm32_uart5_abort_receive()` | 中止旧接收，配合错误恢复；不得在缓冲仍被写入时提前报告安全完成 |
| 同上 | `stm32_uart5_receive_is_ready()` | 返回当前接收是否空闲，供主循环重新启动接收 |
| 同上 | `stm32_usart1_write(data,len)` | 同步消费日志数据；返回后不再读取传入缓冲，false 表示写入失败 |

服务 CAN 发送传入的等待上限为 0 ms，应快速返回发送成功或失败。注册回调的 ctx 长期属于 SDK 单实例，frame 只在本次接收回调有效；应用不能长期保存 frame 指针。

### 13.2 接收分发与并发

CAN 端口将总线映射到 DRIVE/STEER：驱动为标准帧，舵向为扩展帧。接收帧应校验类型和长度，确保不会把 CAN FD 长帧拷入 8 字节结构体。服务将驱动帧暂存后在轮询中解码，将舵向反馈写入受临界区保护的待处理快照。

UART 完成回调调用已注册的 SDK 接收完成函数；该函数读取刚接收字节、记录时间并重新启动接收。错误回调进入原接收恢复路径。移植实现不要在 start_receive 内同步递归触发完成回调，也不要在共享缓冲仍被设备占用时伪造“接收空闲”。

原临界区仅屏蔽本 MCU 中断，不是多任务互斥锁。在有线程或多核的宿主中，应同时保证所有公开 API 串行调用，并让端口保护覆盖实际异步生产者。日志是本工程内的单例，宿主若自行初始化同一 `log` 模块会改变 SDK 的日志配置。

### 13.3 资源与生命周期

| 资源 | 提供方 | 生命周期 |
|---|---|---|
| 配置对象 | 上层 | 初始化时复制，调用结束后可释放 |
| 状态输出对象 | 上层 | get_state 写入副本，不保留其地址 |
| 服务、电机实例和接收缓冲 | SDK 静态存储 | 与当前固件运行期一致，无公开释放接口 |
| UART/CAN 句柄、NVIC、DMA、时钟和 GPIO | 宿主/平台 | 初始化前就绪，SDK 使用期间保持有效 |
| 返回的字符串 | SDK 静态常量 | 不释放、不修改 |
| HAL 回调符号与 CAN 接收注册槽 | 默认平台适配 | 与宿主合并成唯一分发入口 |

## 14. 安装导出与源码交付

### 14.1 三种接入方式怎么选择

| 方式 | 使用场景 | 上层获得什么 |
|---|---|---|
| 仓库内手工编入/EIDE | 保留现有固件工程 | 原源码布局、原板卡入口和 SDK 头文件 |
| CMake add_subdirectory | 上层与 SDK 一起编译 | `wheel_rail::sdk` 及头文件、C11、数学库依赖 |
| CMake 安装后 find_package | 预编译库交付或独立固件工程 | 可重定位的库、公开头文件、移植端口声明和示例 |

静态库必须与最终固件使用相容的 CPU、Thumb、浮点 ABI、C 库及工具链配置。CMake 的包版本检查不验证这些硬件 ABI 条件，分发时应连同编译参数记录和宿主 HAL 配置一起交付。

### 14.2 编译并安装静态库

先按第 7 节选择 HAL 适配或仅核心构建，再执行：

```sh
cmake -S . -B build/sdk-arm -DWHEEL_RAIL_SDK_BUILD_EXAMPLES=ON
cmake --build build/sdk-arm
cmake --install build/sdk-arm --prefix build/sdk-install
```

这里沿用已经配置好的 ARM 构建目录；首次配置仍需要第 7 节的交叉工具链参数。`BUILD_EXAMPLES=ON` 只编译示例静态模块，校验 API 使用，不生成完整 STM32 固件。

默认安装布局：

```text
sdk-install/
├── include/wheel_rail_sdk/
│   ├── wheel_rail_sdk.h
│   ├── chassis_service.h
│   ├── steer_wheel_kinematics.h
│   └── ports/                       # 四份平台接口声明
├── lib/
│   ├── libwheel_rail_sdk.a
│   └── cmake/WheelRailSDK/           # Config、Version、Targets
└── share/wheel_rail_sdk/
    ├── README.md
    └── examples/                     # upper_app.c/.h 和 consumer 工程
```

安装目录中的说明与源码包使用同一份 README；文中源码路径以源码仓库/源码包根为基准。安装包中的文件实际位置以上表为准，源码导航链接用于源码包阅读。GNUInstallDirs 允许宿主修改 lib/include/share 的安装位置。

### 14.3 上层通过 find_package 引用

```cmake
find_package(WheelRailSDK 1.0 CONFIG REQUIRED)
target_link_libraries(my_firmware PRIVATE wheel_rail::sdk)
# 若是核心库，my_firmware 还要提供端口实现。
# 若库含 STM32 适配，my_firmware 还要链接本机 HAL 和生成的外设实现。
```

配置宿主时设置 `-DCMAKE_PREFIX_PATH=<安装目录绝对路径>`。库公开的 include 路径会自动传递，业务代码仍写 `#include "wheel_rail_sdk.h"`。包变量 `WheelRailSDK_VERSION` 表示版本，`WheelRailSDK_WITH_STM32_PORT` 表示该库是否包含默认适配；两者不等于运行时状态查询。

安装目标不导出原源码工程的 HAL target 名称，也不把机器上的 HAL 路径写进公开 include。最终固件的 HAL、端口、链接脚本和硬件编译选项仍由宿主管理。

仓库提供 [独立消费示例](sdk/examples/consumer/CMakeLists.txt)。可复制安装目录到新位置后，在一致的交叉工具链下配置该示例，检查 SDK 能在原仓库以外使用。示例生成上层静态模块；最终硬件链接仍属于固件工程。

### 14.4 生成可分发源码包

打包脚本仅使用 Python 标准库，要求 Python 3.9 或更高版本：

```sh
python sdk/package_sdk.py
python sdk/package_sdk.py --verify build/dist/wheel-rail-sdk-1.0.0-source.zip
```

默认输出 `build/dist/wheel-rail-sdk-1.0.0-source.zip`，版本取自统一头文件。可用 `--output <目标ZIP>` 自定义输出。脚本从固定文件及源码白名单收集文件，排除 `.git`、build、下载工具链、临时文件和未纳入 SDK 的 HAL 生成文件。

压缩包保留当前源码布局，解压后包含唯一根 README、CMake 配置、sdk 头文件/示例/打包脚本、src 实现、tests、原 main 参考和 CubeMX/EIDE 配置。解压后依然可以构建核心库，完整固件仍需生成 HAL。压缩包没有将项目重排为另一套源码目录，因此修改只维护原实现。

额外的 `SDK_MANIFEST.json` 记录版本、交付类型、平台边界和每个源码文件的 SHA-256。脚本打包后立即校验，也可单独执行 verify；缺文件、多文件、重复路径或内容哈希不符会失败。ZIP 条目按固定顺序和固定时间戳写入，相同字节输入可得到一致归档。哈希清单用于核对内容一致性，不是发布者身份签名。

### 14.5 CMake 选项参考

| 选项 | 默认值 | 作用 |
|---|---|---|
| `WHEEL_RAIL_SDK_STM32_PORT` | ON | 包含四个原 STM32 平台实现；OFF 时最终宿主提供端口符号 |
| `WHEEL_RAIL_SDK_HAL_TARGET` | 空 | 源码构建时已存在的 HAL 依赖目标；未设置时需另行提供头文件和宏 |
| `WHEEL_RAIL_SDK_BUILD_EXAMPLES` | OFF | 编译 upper_app 示例模块，不改变 SDK 控制行为 |
| `WHEEL_RAIL_SDK_BUILD_TESTS` | OFF | 编译本机模拟测试；要求关闭 STM32 适配且不是交叉构建 |
| `CMAKE_INSTALL_PREFIX` / `--prefix` | 由 CMake/调用者指定 | 控制安装位置；不影响库的运动参数 |

## 15. 故障码与接入问题排查

### 15.1 故障枚举参考

以下名称的完整前缀为 `CHASSIS_FAULT_`。首次故障锁存后保留该原因，后续 stop 不把原设备故障覆盖成人工停止。

| 名称 | 当前源码触发位置 | 优先检查 |
|---|---|---|
| NONE | 无锁存或显式清除成功 | 结合 ready、command_online 判断是否有有效运行输入 |
| INITIALIZATION | 模型、CAN、电机、遥控或日志装配失败 | 模型参数、HAL 初始化顺序、总线注册与上电状态 |
| DRIVE_TRANSMIT | 原控制路径累计到连续发送失败门限 | FDCAN1 发送 FIFO、收发器、线缆、终端和驱动供电 |
| STEER_TRANSMIT | 原控制路径累计到连续发送失败门限 | FDCAN2、扩展帧支持、发送 FIFO、RS06 供电与 ID |
| STEER_FEEDBACK_TIMEOUT | 等待反馈、准备、回零或切 PP 的阶段超时 | ID 5–8 反馈是否进入回调、是否持续出现新反馈、主循环是否被阻塞 |
| STEER_ANGLE_LIMIT | 原反馈/原始位置目标安全检查不通过 | 角度有限性、反馈圈数、原始目标范围与电机位置配置 |
| DRIVE_FEEDBACK_TIMEOUT | 正常运行路径检查到原 500 ms 驱动反馈超时 | ID 1–4 查询响应、专用总线反馈绑定及接收维护 |
| KINEMATICS | IK、旧轮组修正或目标优化返回异常 | 初始化车型参数、输入速度和运算结果有限性 |
| MANUAL_STOP | 无其他锁存故障时显式 stop | 等待明确恢复请求，不自动恢复 |

清故障操作重新开始反馈观察和回零，并不证明故障根因已消失。若再次出现同一故障，应先处理对应硬件或配置问题。

### 15.2 常见现象

| 现象 | 原因判断 | 处理 |
|---|---|---|
| init 返回 OK，但非零 set_velocity 返回 UNSAFE_STATE | 仍在回零，或模式为 REMOTE | 检查 control_source、ready，继续轮询 |
| 外部模式没有接遥控器却无法链接 huart5 | 默认 UART 源文件仍包含 huart5 引用 | 保留原生成句柄，或关闭默认适配并提供自有端口 |
| 速度 API 返回 OK，状态速度还是旧值 | 还未到下一控制节拍 | 继续 update；不要把状态目标当作即时输入回显 |
| 指令已失联却不停 | 宿主不断重放缓存，或根本没运行 update | 只刷新新业务指令，检查底盘任务是否持续调度 |
| 清故障总是 UNSAFE_STATE | 无锁存、零指令过期，或遥控未出现新的安全帧 | 分模式检查第 5 节条件，不能直接循环 clear |
| 多次 init 返回 UNSAFE_STATE | 已有电机实例活动 | 沿用已初始化实例；故障按恢复流程处理 |
| HAL 回调重复定义 | SDK 平台与宿主都定义了同名回调 | 合并到一个入口，再保留各自分发 |
| 添加 include 仍找不到 HAL 头 | 仅把头文件目录加给了最终应用目标 | 也要给 SDK 编译目标提供 HAL include/宏，或设置 HAL_TARGET |
| find_package 找不到包 | CMAKE_PREFIX_PATH 不是安装根目录 | 指向安装目录或设置 WheelRailSDK_DIR 为 lib/cmake/WheelRailSDK |
| 链接出现浮点 ABI 不兼容 | SDK 与固件的 FPU/float-abi 不一致 | 以一致工具链和 CPU/FPU 参数重建库 |
| 没有里程计、实测底盘速度或逐轮遥测 API | 当前服务只提供原有目标与故障快照 | 在明确需要这些功能时另行扩展，不能从 command 字段推断反馈 |

### 15.3 本次整合完成到什么程度

已经提供源码 SDK、统一 C/C++ 入口、外部速度接入、原遥控兼容、完整接口说明、示例模块、静态库构建/安装/导入、源码包及内容校验。既有电机协议、运动学和平台控制逻辑继续使用同一份源码。尚未提供的能力包括 PC 通信协议、热重配、多实例、后台线程、里程计及完整 STM32 工程生成；它们不属于本轮“少改动、整合 SDK”的交付范围。
