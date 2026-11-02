# Wheel-Rail Integrated Chassis

基于 STM32H723VGT6 的轮轨一体式四舵轮底盘固件与 Linux SDK

- 四舵轮运动学、驱动速度控制与转向回零
- 遥控优先仲裁、自主控制授权、速度超时与故障停车
- 逐轮探轨、保持、释放和取消任务
- 独立 CAN-FD 通信、模块反馈、IMU 接入和时间同步

## 目录

| 目录 | 内容 |
|---|---|
| [mcu/](mcu/README.md) | STM32 固件、接线、参数与传感器适配 |
| [sdk/](sdk/README.md) | Linux SocketCAN 库、安装与控制示例 |
| [docs/](docs/轮轨导航_上位机与STM32接口说明.md) | CAN 报文布局、控制语义与数据有效性 |

## 快速开始

### Linux SDK

需要 Linux、C++17 编译器和 CMake

```sh
cmake -S sdk -B sdk/build -DCMAKE_BUILD_TYPE=Release
cmake --build sdk/build
```

配置 CAN-FD 接口后监视状态：

```sh
sdk/build/wheel_rail_monitor can0
```

安装、集成和运动控制见 [SDK 使用说明](sdk/README.md)

### MCU 固件

使用 STM32CubeMX 打开 `mcu/wheel_rail.ioc`，生成 HAL、外设初始化、启动文件和链接脚本，再通过 `mcu/wheel_rail.code-workspace` 中的 EIDE 配置构建

FDCAN1/FDCAN2 连接驱动与转向电机，FDCAN3 连接上位机；上电默认禁能，启用前需完成急停输入适配、电机方向核对和运动参数标定；详见 [硬件与配置](mcu/README.md)
