# Wheel-Rail Integrated Chassis

基于 STM32H723VGT6 的轮轨复合四舵轮底盘电控工程；项目按
`docs/Embedded-Electronic-Control-Standard/` 的完整六层模式组织，`OLD/` 保存迁移前的可用旧架构代码，仅作为行为对照，不参与构建

## 硬件组成

- MCU：STM32H723VGT6（Cortex-M7）
- 驱动轮：4 台本末总线电机，FDCAN1，节点 ID 1～4
- 舵向轮：4 台 RS06，FDCAN2，节点 ID 5～8
- 遥控器：FS-iA10B，UART5，iBUS 115200-8-N-1
- 日志：USART1，9600-8-N-1，默认阻塞输出
- FDCAN1：500 kbit/s；FDCAN2：1 Mbit/s

## 软件架构

```text
src/
├── app/       # entry_init / entry_loop，整机入口
├── service/   # 底盘组装、周期控制、状态缓存和安全策略
├── device/    # iBUS、驱动电机和 RS06 协议与反馈缓存
├── domain/    # 四舵轮运动学和旧版 ID 5/7 几何修正
├── infra/     # 与硬件无关的日志设施
└── platform/  # STM32 HAL 的 FDCAN、UART、Tick、临界区和板级适配
```

依赖方向为 `app -> service -> device/domain/infra/platform`；`device` 通过 PortOps 接收平台能力；只有 `platform/` 和 CubeMX 生成的 `Core/` 可以包含或调用 STM32 HAL

## 兼容参数

- 轴距：0.725 m
- 轮距：0.730 m
- 轮半径：0.0215 m
- 单轮最大线速度：0.45 m/s
- 驱动转速范围：-210～210 RPM
- 驱动斜坡：每 10 ms 改变 5 RPM
- 遥控通道、死区、挡位、VRB 使能条件、电机方向、协议字节和 RS06 初始化顺序均以 `OLD/` 为兼容基线
- 旧版 ID 5/7 几何修正会在运动学 0.45 m/s 归一化之后重算对应轮速，最终由驱动层 ±210 RPM 限幅；这是为保持可运行旧版行为而保留的兼容债务

## 初始化与运行

CubeMX 生成的 `main.c` 完成 GPIO、DMA、FDCAN、UART 和定时器初始化后调用：

```c
entry_init();

while (1) {
    entry_loop();
}
```

service 初始化时依次组装运动学、FDCAN1/驱动电机、FDCAN2/RS06、UART5/iBUS 和 USART1 日志；控制调度不使用周期性阻塞延时，service 使用系统 tick 每 10 ms 执行一次控制更新；配置的诊断日志仍通过 USART1 同步发送

## 编译与烧录

1. 使用 VS Code 打开 `wheel_rail.code-workspace`
2. 安装 Embedded IDE 扩展，并选择 EIDE 的 `Debug` 或 `Release` target
3. 使用项目配置中的 ARM GCC 工具链构建
4. 按 `.eide/eide.yml` 配置通过 OpenOCD/J-Link 烧录；烧录前确认目标芯片与调试器配置

`OLD/` 不在 EIDE 的 `srcDirs` 中，不会进入固件

## 联调与安全

- 上电默认目标为零，完成初始化后才接受遥控速度
- 遥控失联或 VRB 关闭时仅下发零速度，保持旧版不主动失能的行为
- 初始化失败、连续 3 个控制周期发送失败、驱动接收队列溢出、驱动反馈超时或运动学异常会锁存故障
- 故障锁存后，驱动轮立即下发零速，RS06 下发 stop；不会自动恢复
- `chassis_service_stop()` 会锁存人工停止，防止下一控制周期自动恢复输出
- `chassis_service_fault_clear()` 采用非阻塞两步确认：首次调用建立新遥控帧基线，收到更新的有效 iBUS 帧且 VRB 速度使能已关闭后，再次调用才会清除故障；重新上电也会清除锁存
- 禁止在未确认急停链路、机械安全边界和低速工况前直接上真实负载

## 当前状态

架构和代码已从 `OLD/` 迁移到六层结构；按照本次任务要求，没有新增测试，也没有执行编译或真实硬件验证，因此当前状态为“静态重构完成、运行未验证”，不满足标准 Definition of Done 中的测试记录与安全路径实测条款

旧版 `BlueSerial` 与日志共用 USART1 且在可运行版本中默认关闭；其文本、数字和格式化发送能力已统一由 `infra/log` 与 `platform/stm32_uart_port` 承担，UART 接收仍保持默认关闭
