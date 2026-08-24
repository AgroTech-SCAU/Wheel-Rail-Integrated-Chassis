# Wheel-Rail Integrated Chassis

基于 STM32H723VGT6 的轮轨复合四舵轮底盘实时控制工程

项目采用六层嵌入式架构组织底盘控制 遥控输入 电机协议 运动学 日志和 STM32 HAL 适配

## 功能特性

- 四舵轮底盘逆运动学解算
- FS-iA10B iBUS 遥控输入与链路超时检测
- 四节点本末驱动电机速度控制与反馈轮询
- 四节点 RS06 舵向电机位置控制
- 驱动速度限幅与每周期斜坡控制
- CAN 发送错误与驱动反馈超时监控
- 故障锁存 安全停止和显式故障清除
- USART1 分级日志输出
- CubeMX 用户代码区应用入口

## 硬件

| 模块 | 配置 |
|---|---|
| MCU | STM32H723VGT6 Cortex-M7 |
| 驱动电机 | 本末总线电机 4 台 节点 ID 1 至 4 |
| 舵向电机 | RS06 4 台 节点 ID 5 至 8 |
| 驱动总线 | FDCAN1 500 kbit/s |
| 舵向总线 | FDCAN2 1 Mbit/s |
| 遥控器 | FS-iA10B UART5 iBUS 115200-8-N-1 |
| 日志 | USART1 9600-8-N-1 |

## 软件架构

```text
src/
├── app/       # 应用入口
├── service/   # 底盘装配 控制调度 状态与安全策略
├── device/    # 遥控器 驱动电机和舵向电机协议
├── domain/    # 四舵轮运动学
├── infra/     # 通用日志基础设施
└── platform/  # STM32 FDCAN UART Tick 临界区与板级适配
```

依赖方向保持为以下结构

```text
app -> service -> device/domain/infra/platform
```

`app` 只调用 `service` 层

`device` 通过 PortOps 接收平台能力

`domain` 和 `infra` 不直接依赖 STM32 HAL

`platform` 是业务源码访问 STM32 HAL 和 CubeMX 外设句柄的唯一出口

## 控制参数

| 参数 | 数值 |
|---|---:|
| 轴距 | 0.725 m |
| 轮距 | 0.730 m |
| 轮半径 | 0.0215 m |
| 单轮线速度限制 | 0.45 m/s |
| 驱动转速范围 | -210 至 210 RPM |
| 驱动斜坡 | 每 10 ms 改变 5 RPM |
| 控制周期 | 10 ms |
| 遥控链路超时 | 200 ms |
| 驱动反馈超时 | 200 ms |

## 启动与调度

CubeMX 完成 GPIO DMA FDCAN UART 和定时器初始化后调用应用入口

```c
entry_init();

while (1) {
    entry_loop();
}
```

`entry_init()` 初始化底盘运动学 驱动总线 舵向总线 遥控接收和日志输出

`entry_loop()` 持续维护接收队列并按 10 ms 周期执行遥控映射 运动学解算 电机命令下发 反馈轮询和安全检查

## 构建与烧录

1. 使用 VS Code 打开 `wheel_rail.code-workspace`
2. 安装 Embedded IDE 扩展
3. 在 EIDE 中选择 `Debug` 或 `Release` target
4. 使用 ARM GCC 工具链构建固件
5. 通过 OpenOCD 或 J-Link 烧录目标板

工程构建配置使用 `.eide/eide.yml` 文件

CubeMX 硬件配置使用 `wheel_rail.ioc` 文件

## 核心接口

```c
ChassisServiceStatus chassis_service_init(void);
ChassisServiceStatus chassis_service_update(void);
ChassisServiceStatus chassis_service_stop(void);
ChassisServiceStatus chassis_service_fault_clear(void);
ChassisServiceStatus chassis_service_get_state(ChassisServiceState* out);
```

`chassis_service_stop()` 锁存人工停止并向全部执行器发送安全停止命令

`chassis_service_fault_clear()` 需要新的有效遥控帧且 VRB 速度使能关闭后才能清除锁存故障

## 安全行为

- 上电目标速度保持为零
- 遥控失联或 VRB 关闭时下发零速度
- 初始化失败会进入锁存故障
- 连续三次控制发送失败会进入锁存故障
- 驱动接收队列溢出会进入锁存故障
- 驱动反馈超时会进入锁存故障
- 运动学输入或模型异常会进入锁存故障
- 故障锁存后驱动轮下发零速且 RS06 下发停止命令
- 停止命令发送失败时会在后续轮询中继续尝试
- 锁存故障不会自动恢复
