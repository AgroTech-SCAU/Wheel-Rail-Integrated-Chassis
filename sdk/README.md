# 轮轨底盘 Linux SDK

独立的 Linux C++17 库，通过 SocketCAN 与 MCU 通信，提供协议编解码、状态订阅、控制权申请、速度指令、逐轮转向任务和时间同步；不依赖 ROS 或 STM32 HAL

- [通信接口](../docs/轮轨导航_上位机与STM32接口说明.md)
- [MCU 配置与构建](../mcu/README.md)
- [只读监视示例](examples/monitor.cpp) / [控制示例](examples/control.cpp)

## 构建与安装

需要 Linux、C++17 编译器和 CMake；CAN 接口与驱动须支持 CAN-FD

在仓库根目录执行：

```sh
cmake -S sdk -B sdk/build -DCMAKE_BUILD_TYPE=Release
cmake --build sdk/build
cmake --install sdk/build --prefix "$PWD/sdk/build/install"
```

应用通过安装前缀查找库：

```cmake
find_package(wheel_rail CONFIG REQUIRED)
target_link_libraries(my_app PRIVATE wheel_rail::sdk)
```

配置应用时传入 `-DCMAKE_PREFIX_PATH=/path/to/install`；公开头文件位于 `wheel_rail/` 下

## 连接底盘

上位机使用独立 CAN-FD 总线，仲裁段 1 Mbit/s、数据段 4 Mbit/s，启用 BRS；接口名称和接线应与实际设备一致

```sh
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 4000000 fd on
sudo ip link set can0 up
ip -details link show can0
```

SDK 只使用已配置的接口；网卡配置、收发器和终端电阻由部署端准备

```sh
# 监视整车、四个轮组和 IMU 状态
sdk/build/wheel_rail_monitor can0

# 申请平地模式，以 vx=0.1 m/s、vy=0、wz=0 行驶 1 秒后停车并释放控制权
sdk/build/wheel_rail_control can0 request-autonomy free 0.1 0 0 1
```

控制示例等待新鲜整车状态、控制 ACK 和实际使能，再以 50 Hz 发送速度；运行运动示例前须完成 MCU 参数配置，并确认车辆运动空间

## 应用接入

`Client` 由同一线程串行调用；应用持续执行 `poll()` 接收遥测并推进状态和 ACK 超时，SDK 不创建后台线程

| 接口 | 用途 |
| --- | --- |
| `open()` / `close()` | 打开或关闭本地 SocketCAN 接口 |
| `poll()` / `snapshot()` | 接收事件并读取带新鲜度标记的状态副本 |
| `request_autonomy()` / `release_control()` | 申请或释放自主控制权 |
| `send_velocity()` | 发送一次速度指令；单位为 m/s、m/s、rad/s |
| `steering_task()` | 对模块掩码指定的轮组执行 PROBE、HOLD、RELEASE 或 CANCEL |
| `quick_stop()` / `clear_fault()` | 请求软件快速停车或清除故障 |
| `request_time_sync()` / `to_host_time()` | 发起对时并换算 MCU 单调采样时间 |

最短控制流程为 `open → poll → request_autonomy → poll 至授权生效 → 周期发送速度并 poll → 零速度 → release_control`；每次发送均须检查返回的 `error`，完整用法见 [control.cpp](examples/control.cpp)

## 控制与状态语义

- 本地发送成功仅表示帧已交给网络栈；`0x188` 确认 MCU 是否接受控制或任务事件，实际控制源、模式与使能以 `0x180` 为准，任务进展以 `0x187` 为准
- 非零速度和非 CANCEL 任务要求控制申请成功、新鲜状态确认自主控制及使能、电机就绪且无急停或故障；零速度和 CANCEL 可在未授权时发送
- 速度没有逐帧 ACK，建议 50 Hz 发送，有效期为 1～200 ms；控制申请不能代替速度保活，轨内模式要求 `vy=0`、`wz=0`
- 遥控接管、状态过期、MCU 重启或链路故障会使旧授权失效；恢复后重新申请控制权并提交新指令，SDK 不自动重发或恢复运动
- ACK 超时表示结果未知；应用根据最新状态决定后续操作，软件快速停车不能替代物理急停
- `snapshot()` 返回副本，`fresh` 只反映取副本时的接收新鲜度；传感器是否有效仍须检查有效位，未安装或未标定的数据可为 `NaN`
- 各遥测帧保留独立采样时间；时间同步由应用主动请求，建议约 1 Hz；默认状态超时 100 ms、ACK 超时 200 ms、对时映射有效期 5 s
- 每次 `open()` 默认生成新会话；同一底盘应由单一网关发送控制命令，SocketCAN 本身不提供进程独占控制
