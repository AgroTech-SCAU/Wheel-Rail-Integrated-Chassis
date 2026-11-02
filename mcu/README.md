# MCU 固件

基于 STM32H723VGT6 的四舵轮底盘控制程序，提供遥控与上位机控制、回零、逐轮转向任务、电机保护及 CAN-FD 遥测

上位机接口见 [通信协议](../docs/轮轨导航_上位机与STM32接口说明.md)，Linux 接入见 [SDK](../sdk/README.md)

## 目录

| 路径 | 用途 |
|---|---|
| `Core/Src/main.c` | CubeMX 主入口 |
| `wheel_rail.ioc` | 芯片与外设配置 |
| `src/app/` | 板级输入、控制和通信主循环 |
| `src/service/` | 控制权、速度有效期、任务、故障和遥测调度 |
| `src/device/` | iBUS、本末驱动电机、RS06 转向电机及上位机协议 |
| `src/domain/` | 四舵轮运动学与回零 |
| `src/platform/` | STM32 HAL 适配 |
| `src/infra/` | 日志 |

## 硬件接口

| 用途 | 外设与引脚 | 配置 |
|---|---|---|
| 驱动电机 | FDCAN1，PD0 RX / PD1 TX | 经典 CAN 500 kbit/s，节点 1–4 |
| 转向电机 | FDCAN2，PB5 RX / PB6 TX | 经典 CAN 1 Mbit/s，RS06 节点 5–8 |
| 上位机 | FDCAN3，PD12 RX / PD13 TX，AF5 | CAN-FD + BRS，仲裁 1 Mbit/s、数据 4 Mbit/s |
| 遥控接收器 | UART5，PD2 RX / PB13 TX | iBUS，115200 bit/s，RX 反相 |
| 日志 | USART1，PA9 TX / PA10 RX | 9600 bit/s，发送等待上限 2 ms |
| 接收器供电 | PC15 | 控制遥控接收器 5V，停车时保持供电 |
| 单调时钟 | TIM5 | 1 MHz 自由运行，累计为 64 位微秒 |
| 启动标识 | RNG | 每次启动生成非零 `boot_id` |

上位机总线需要独立 CAN-FD 收发器、相应接线和终端电阻；电机总线与上位机总线分开连接

三路 FDCAN 共享 2560 个 32 位 Message RAM 字；FDCAN3 偏移为 1536 字，配置 4 个标准过滤器、32 个 64 byte RX 元素和 16 个 64 byte TX 元素，占用 868 字，修改时需同时核对三路分配

## 配置与启用

在 [entry.c](src/app/entry.c) 中取得 `chassis_service_default_config()` 后设置底盘参数；默认参数对应轴距 0.725 m、轮距 0.730 m、地面轮有效半径 0.0215 m 和单轮速度上限 0.45 m/s，移植到其他底盘前应实测校准

| 配置项 | 含义与启用条件 |
|---|---|
| `model` | 车体几何、地面轮有效半径及单轮速度上限 |
| `drive_direction` / `steer_direction` | FL、FR、RR、RL 电机正方向映射，各项为 `+1` 或 `-1` |
| `yaw_bias` | 平移偏航前馈补偿；应用入口包含参考标定值 |
| `legacy_steer_57_correction` | ID 5/7 轮组的安装几何修正，修正后仍受轮速上限约束 |
| `rail_radius_m` | 轨轮有效半径；默认 0，未标定时拒绝 ON_RAIL |
| `probe_direction` / `probe_min_angle` / `probe_max_angle` | 逐轮探轨方向及机械角界限 |
| `probe_max_speed_rad_s` / `probe_hard_torque_nm` / `probe_max_temperature_c` | 探轨速度上限、硬停转矩和硬停温度；默认未标定，拒绝 PROBE |
| `require_estop` | 默认 `true`，急停输入有效且未触发才允许启用 |

上电保持零速度和转向 STOP，收到有效控制授权后才使能执行器并回零；控制优先级为物理急停及硬故障、遥控、上位机自主控制

速度命令按实际接收时间计算有效期，上限 200 ms；超时停车，控制事件不续期；故障、遥控接管或释放、会话变化会清除旧运动，恢复需要重新授权和新的速度命令

ON_RAIL 仅允许有界正反向 `vx`，`vy` 和 `wz` 必须为零；PROBE、HOLD、RELEASE 和 CANCEL 按轮掩码执行，任务期间普通运动学不覆盖转向目标；任务超时、过矩、过温和机械越限会锁存停车

## 板级输入与传感器

板级采集代码通过 `stm32_board_submit_inputs()` 提交急停和电池测量；初始化时 `estop_valid` 与 `battery_valid` 均为 `false`，需要按电路接入 GPIO、ADC 或其他采集驱动；采集失效时应清除对应有效标志，电池测量无效时遥测使用 NaN

IMU 驱动通过 `host_link_service_submit_imu()` 提交真实样本，`sample_time_us` 使用采样时刻的 TIM5 单调微秒时间；可信融合姿态通过 `host_link_service_submit_orientation()` 提交；未接入驱动或样本超过 100 ms 时不标为有效，仅发送新鲜且可信的姿态

RS06 反馈包含角度、角速度、转矩、温度、故障位和运行模式；整车就绪要求转向反馈新鲜且为 Run 状态；本末查询提供转速与错误码；未采集的电流、驱动位置、驱动转矩及驱动温度使用 NaN，并清除相应有效标志

## 固件构建

需要 STM32CubeMX、ARM GCC 和 EIDE；仓库保留业务代码及外设配置，HAL/CMSIS、外设初始化文件、启动文件和链接脚本由 CubeMX 生成

1. 打开 `wheel_rail.ioc`，在 `mcu/` 下生成工程与 HAL 文件
2. 确认生成 FDCAN3 GPIO/IRQ、TIM5、RNG/HSI48 及电机、遥控所需外设，保留 `main.c` 中的 USER CODE 调用
3. 打开 `wheel_rail.code-workspace`，为 EIDE 配置 ARM GCC 和调试器后构建、烧录
4. 接入有效急停输入，检查 CAN-FD 收发与时间基准；架空核对电机方向、回零和停车，再标定轨轮半径与探轨参数
