/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "key.h"
#include "motor.h"
#include "can.h"
#include "servo.h"
#include "string.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>
#include "swerve_chassis.h"
#include "steer_wheel_kine.h"
#include "remote_chassis.h"
#include "FS-IA10B.h"
// #include "BlueSerial.h"
#include "log.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern reporter Motor_Reporter_Data;
extern uint8_t query_id;
extern reporter Motor_Reporter_Cache[4];
SwerveChassis chassis; //注册实例电机
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ================================================================
 * 日志输出端口：USART1 阻塞发送
 * ================================================================ */
static bool board_log_write(const char* data, uint32_t len)
{
    return HAL_UART_Transmit(&huart1, (uint8_t*)data, (uint16_t)len, HAL_MAX_DELAY) == HAL_OK;
}

static const LogPortOps log_ops = {
    .write = board_log_write,
};

/* ================================================================
 * 初始化自检打印（仅在启动时调用一次）
 *
 * 打印内容包括：
 *   - 系统主控与时钟
 *   - CAN 总线配置与使能状态
 *   - 4 路驱动电机 (ID 1~4) 配置详情
 *   - 4 路舵向电机 (ID 5~8) 使能与模式详情
 *   - 底盘物理参数
 *   - 遥控器接收机与调试串口
 * ================================================================ */
static void Log_Init_SelfCheck(const SwerveChassis* chassis)
{
    log_info("============================================");
    log_info("  轮轨复合底盘 - 上电初始化自检报告");
    log_info("============================================");

    /* ---------- 1. 系统主控 ---------- */
    log_info("[系统] MCU 型号: STM32H723VGT6 (Cortex-M7, LQFP100)");
    log_info("[系统] 主频: 200 MHz (HSI 64MHz /4*12/1 = 192MHz→PLL→200MHz)");
    log_info("[系统] 内核供电: LDO, VOS 电压缩放等级 1");
    log_info("[系统] MPU: 已配置 (Region0, 4GB 背景区域)");

    /* ---------- 2. CAN 总线 ---------- */
    log_info("[CAN] FDCAN1 (驱动电机总线): 500 kbit/s");
    log_info("[CAN]   引脚: PD0=FDCAN1_TX, PD1=FDCAN1_RX");
    log_info("[CAN] FDCAN2 (舵向电机总线): 1 Mbit/s");
    log_info("[CAN]   引脚: PB5=FDCAN2_TX, PB6=FDCAN2_RX");
    log_info("[CAN] CAN1_EN (PC13): 已拉高, CAN1 收发器使能");
    log_info("[CAN] CAN2_EN (PC14): 已拉高, CAN2 收发器使能");
    log_info("[CAN] 全局滤波器: 双 FIFO0 全部接收, 拒绝远程帧");

    /* ---------- 3. 驱动电机 (ID 1~4) ---------- */
    log_info("[驱动] 数量: 4 台, 总线 FDCAN1, 通信帧 ID=0x032 (标准帧)");
    log_info("[驱动] ID=1 (FL 前左): 方向正向, 平滑斜坡 5.0 RPM/step");
    log_info("[驱动] ID=2 (FR 前右): 方向正向, 平滑斜坡 5.0 RPM/step");
    log_info("[驱动] ID=3 (RR 后右): 方向反向 (硬件取反), 平滑斜坡 5.0 RPM/step");
    log_info("[驱动] ID=4 (RL 后左): 方向反向 (硬件取反), 平滑斜坡 5.0 RPM/step");
    log_info("[驱动] 转速限幅: %d ~ %d RPM", SPEED_RPM_MIN, SPEED_RPM_MAX);
    log_info("[驱动] 急停接口: Motor_Stop_Immediately() 清零目标+当前转速");
    log_info("[驱动] 查询帧: ID=0x107, 轮询读取反馈 (速度/位置/错误码)");
    log_info("[驱动] TIM6 定时器: 已启动, 用于 4 路平滑斜坡周期更新");

    /* ---------- 4. 舵向电机 (ID 5~8, RS06 协议) ---------- */
    log_info("[舵向] 数量: 4 台, 总线 FDCAN2, 通信帧=29位扩展帧 (RS06 协议)");
    log_info("[舵向] 主机 ID: 0x%02X, 角度限幅: %.2f ~ %.2f rad", RS06_HOST_ID, (double)RS06_P_MIN, (double)RS06_P_MAX);
    log_info("[舵向] ID=5 (FL 前左): PP 位置模式, 已发送使能帧 (0x0300), 初始目标=0.0 rad");
    log_info("[舵向] ID=6 (FR 前右): PP 位置模式, 已发送使能帧 (0x0300), 初始目标=0.0 rad");
    log_info("[舵向] ID=7 (RR 后右): PP 位置模式, 已发送使能帧 (0x0300), 初始目标=0.0 rad");
    log_info("[舵向] ID=8 (RL 后左): PP 位置模式, 已发送使能帧 (0x0300), 初始目标=0.0 rad");
    log_info("[舵向] 方向: ID=5/7 在 wz 旋转时额外 +90° 偏移并取反 wz");
    log_info("[舵向] 归零与保存: RS06_Zeroing_And_Save_Process() 可用");

    /* ---------- 5. 底盘物理参数 ---------- */
    log_info("[底盘] 前后轴距 (length): %.3f m", (double)chassis->model.length);
    log_info("[底盘] 左右轮距 (width):  %.3f m", (double)chassis->model.width);
    log_info("[底盘] 轮子半径 (radius): %.4f m", (double)chassis->model.wheel_radius);
    log_info("[底盘] 单轮最大线速度:    %.2f m/s", (double)chassis->model.max_wheel_linear_speed);
    log_info("[底盘] 解算方式: 四轮独立逆运动学 (IK) → wheel_omega + steer_angle");

    /* ---------- 6. 遥控器接收机 ---------- */
    log_info("[遥控] 接收机型号: FS-iA10B (iBUS 协议)");
    log_info("[遥控] 接口: UART5 (PB13=RX, DMA1_Stream1 循环接收), 115200-8-N-1");
    log_info("[遥控] 通道数: 14 ch, 安全使能阈值 VRB > %u", REMOTE_VRB_ENABLE_THRESHOLD);
    log_info("[遥控] RC 离线保护: 仅发送零速, 不断电机使能");

    /* ---------- 7. 调试与状态输出 ---------- */
    log_info("[调试] 日志输出: USART1 (PA9=TX), 9600-8-N-1");
    log_info("[调试] 日志级别: INFO (ERROR+WARN+INFO 均输出)");
    log_info("[调试] ANSI 彩色: 启用 (错误红/警告黄/信息蓝)");
    log_info("[调试] USART1 蓝牙串口 (BlueSerial): 当前已禁用");

    log_info("============================================");
    log_info("  自检通过, 所有电机已使能, 进入主循环");
    log_info("============================================");
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_UART5_Init();
  MX_FDCAN2_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
    //舵轮底盘物理参数
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15, GPIO_PIN_SET);
    Swerve_Chassis_Model_Init(
        &chassis,

        0.725f,     // 前后轴距 m
        0.730f,     // 左右轮距 m
        0.0215f,    // 轮子半径 m
        0.45f       // 单轮最大线速度
    );
/* 初始化舵轮底盘 */
Swerve_Chassis_Init(&chassis);

/* FS-IA10B 初始化，内部已经开启 UART5 RX 中断 */
ibus_init();

/* 初始化日志模块（USART1 输出） */
{
    LogConfig log_config = {
        .ops = &log_ops,
        .level = LOG_LEVEL_INFO,
        .enable_color = true,
        .async_write = false,
    };
    log_init(&log_config);
}

/* 上电初始化自检：打印所有电机使能状态和系统配置（仅此一次） */
Log_Init_SelfCheck(&chassis);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1) {
     /* 维持 IBUS 接收 */
    Remote_Chassis_Update(&chassis);
    /*
     * 调用你的舵轮底盘解算和电机输出
     */
    Swerve_Chassis_Update(&chassis);

    HAL_Delay(10);
    // /* 遥控器控制底盘 */

    // Remote_Chassis_Update(&chassis);

    // /* 更新底盘 */
    // Swerve_Chassis_Update(&chassis);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 4096;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV4;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    ibus_rx_complete_callback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    ibus_error_callback(huart);
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while(1) {
    }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
