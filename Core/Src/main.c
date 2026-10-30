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
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>

#include "GM6020.h"
#include "MasterCan.h"
#include "VESC.h"
#include "angle_opt.h"
#include "bsp_can.h"
#include "setpoint_ramp.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*
 * ============================================================================
 * 舵轮分控板顶层应用说明
 * ============================================================================
 *
 * 数据流：
 *   主控 CAN2 控制帧
 *       -> MasterCan 解码并维护链路状态
 *       -> 主控坐标映射
 *       -> AngleOpt 选择最短物理等效舵向和 ERPM 符号
 *       -> SetpointRamp 平滑 GM6020 目标角
 *       -> GM6020 位置环 + VESC 内置速度环
 *
 * 运行节拍：
 *   - CAN 接收中断：只完成当前报文的解码和状态更新。
 *   - TIM6 1 kHz：链路/执行器安全检查、命令应用、位置斜坡和驱动控制。
 *   - 主循环：10 ms 更新 LED，20 ms 输出一次 UART 遥测。
 *
 * 故障策略：
 *   - 主控链路、GM6020、VESC 任一失效时，整轮立即失能。
 *   - 故障后清除待处理控制帧；恢复时必须收到一帧新的有效控制命令。
 *   - 失能时保留最后目标和故障信息，便于串口诊断。
 */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* 设备身份：主控轮号、GM6020 ID、机械零位和 VESC ID。 */
#define APP_MASTER_WHEEL_ID          1U
#define APP_GM6020_ID                1U
#define APP_GM6020_ZERO_RAW          1386U
#define APP_VESC_CONTROLLER_ID       67U

/*
 * 主控角度到舵轮本地坐标的线性映射：
 *   target_wheel = SIGN * target_master + OFFSET
 * 当前安装方向与主控坐标同向，因此 sign=+1、offset=0。
 */
#define APP_MASTER_ANGLE_SIGN        1.0f
#define APP_MASTER_ANGLE_OFFSET_DEG  0.0f

/*
 * 定时参数：
 * - 启动后 500 ms 内四个 LED 常亮，并强制禁止执行器使能；
 * - 遥测周期 20 ms 对应 50 Hz；
 * - LED 周期 10 ms，避免主循环高频重复写 GPIO。
 */
#define APP_SELF_TEST_MS             500U
#define APP_TELEMETRY_MS             20U
#define APP_LED_UPDATE_MS            10U

/* 测试 LED 映射，所有 LED 均为低电平点亮。 */
#define APP_LED_MASTER_PIN           GPIO_PIN_4
#define APP_LED_VESC_PIN             GPIO_PIN_5
#define APP_LED_ENABLE_PIN           GPIO_PIN_6
#define APP_LED_GM6020_PIN           GPIO_PIN_7

/*
 * UART 遥测故障位。多个故障可同时出现，串口输出的是这些位按位或的结果。
 * TX 故障位只作为诊断提示，不直接参与电机安全判断。
 */
#define APP_FAULT_MASTER_LINK        0x00000001U
#define APP_FAULT_NO_CONTROL         0x00000002U
#define APP_FAULT_GM6020_OFFLINE     0x00000004U
#define APP_FAULT_VESC_OFFLINE       0x00000008U
#define APP_FAULT_NEED_FRESH         0x00000010U
#define APP_FAULT_GM6020_TX          0x00000020U
#define APP_FAULT_VESC_TX            0x00000040U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* 驱动和协议模块的持久上下文，由 main.c 统一持有并传入各模块。 */
static GM6020_t g_gm6020;
static VESC_t g_vesc;
static MasterCan_t g_master;
static SetpointRamp_t g_target_ramp;

/* TIM6 维护的自由运行毫秒时钟，支持 uint32_t 自然回绕。 */
static volatile uint32_t g_ms;

/*
 * CAN2 中断写入、TIM6 消费的新控制帧标志。
 * 角度候选只在收到新控制帧时重新选择，不能在 1 kHz 周期中反复计算。
 */
static volatile uint32_t g_control_pending;

/* 应用层在线状态；由 TIM6 根据最近反馈时间和驱动超时参数计算。 */
static volatile uint8_t g_gm6020_online;
static volatile uint8_t g_vesc_online;

/* 整轮状态：active 表示当前使能运行；need_fresh 表示必须等待新控制帧。 */
static volatile uint8_t g_wheel_active;
static volatile uint8_t g_need_fresh_control;

/* 最近一次命令转换结果，供 UART 遥测和故障分析使用。 */
static volatile uint8_t g_angle_reversed;
static volatile float g_mapped_target_deg;
static volatile float g_final_target_angle_deg;
static volatile int32_t g_final_target_erpm;

/*
 * UART1 非阻塞发送状态：
 * g_uart_busy 由主循环置位，由发送完成中断清零。
 * 其余周期调度时间戳只由主循环读写。
 */
static volatile uint8_t g_uart_busy;
static char g_uart_buffer[384];
static uint32_t g_last_telemetry_ms;
static uint32_t g_last_led_ms;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief 计算用于遥测显示的时间差。
 * @param now_ms 当前毫秒时间。
 * @param timestamp_ms 待检查事件的时间戳。
 * @return 时间差；时间戳为 0 时返回 0xFFFFFFFF，表示尚未收到。
 */
static uint32_t App_AgeMs(uint32_t now_ms, uint32_t timestamp_ms)
{
    if (timestamp_ms == 0U)
    {
        return 0xFFFFFFFFU;
    }

    return (uint32_t)(now_ms - timestamp_ms);
}

/**
 * @brief 计算无符号毫秒时间差，不把时间戳 0 当作无效值。
 * @note 用于在线判断；设备可能在系统启动后的第 0 ms 收到第一帧。
 */
static uint32_t App_ElapsedMs(uint32_t now_ms, uint32_t timestamp_ms)
{
    return (uint32_t)(now_ms - timestamp_ms);
}

/**
 * @brief 将任意角度归一化到 [0, 360)。
 * @note 主控坐标映射后调用；AngleOpt 内部仍会再次执行归一化。
 */
static float App_Wrap360(float angle_deg)
{
    angle_deg = fmodf(angle_deg, 360.0f);
    if (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

/**
 * @brief 同时失能 GM6020 和 VESC，并进入“等待新控制帧”状态。
 * @note 不清除主控最后解码值、最终角度目标和 ERPM，故障时仍可通过遥测观察。
 */
static void App_DisableWheel(void)
{
    if (g_gm6020.is_enabled)
    {
        GM6020_Disable(&g_gm6020);
    }

    if (g_vesc.is_enabled)
    {
        VESC_Disable(&g_vesc);
    }

    g_wheel_active = 0U;
    g_need_fresh_control = 1U;
}

/**
 * @brief 应用 CAN2 收到的新控制帧。
 *
 * 处理顺序：
 * 1. 把主控角度映射到舵轮本地坐标；
 * 2. 只在本函数中执行一次 AngleOpt 候选选择；
 * 3. 首次运行时使能两个执行器，并把斜坡起点设为当前 GM6020 角度；
 * 4. 写入最终角度和 ERPM 目标。
 *
 * @note 只能在 TIM6 回调中调用，确保与 1 kHz 控制步骤处于同一上下文。
 * @note 后续控制帧不会重置斜坡或 PID，以保持已调好的动态行为。
 */
static void App_ApplyPendingControl(void)
{
    AngleOptResult_t result;
    float mapped_target_deg;

    /* 当前安装方向与主控坐标同向；保留 sign/offset 供后续机械调整。 */
    mapped_target_deg = App_Wrap360(
        (APP_MASTER_ANGLE_SIGN * g_master.target_angle_deg) +
        APP_MASTER_ANGLE_OFFSET_DEG);

    /* 角度候选一旦选定就保持，直到下一帧主控控制命令到达。 */
    AngleOpt_Select(g_gm6020.angle_deg,
                    mapped_target_deg,
                    g_master.drive_erpm,
                    &result);

    /* 首次进入活动状态时，确保目标从实际当前位置连续开始。 */
    if (!g_wheel_active)
    {
        GM6020_Enable(&g_gm6020);
        VESC_Enable(&g_vesc);
        SetpointRamp_Reset(&g_target_ramp, g_gm6020.angle_deg);
        g_wheel_active = 1U;
    }

    g_mapped_target_deg = mapped_target_deg;
    g_final_target_angle_deg = result.target_angle_deg;
    g_final_target_erpm = result.target_erpm;
    g_angle_reversed = result.reversed;

    SetpointRamp_SetDesired(&g_target_ramp,
                            result.target_angle_deg);
    VESC_SetTargetErpm(&g_vesc, result.target_erpm);

    g_control_pending = 0U;
    g_need_fresh_control = 0U;
}

/**
 * @brief 1 kHz 应用控制步骤。
 *
 * 本函数完成链路更新、执行器在线判断、整轮故障闭合、新命令应用、
 * 位置斜坡更新和两个执行器的控制输出。它只能由 TIM6 周期中断调用。
 */
static void App_ControlStep(void)
{
    uint8_t master_ready;
    uint8_t actuators_online;

    MasterCan_Update(&g_master, g_ms);

    /* 直接根据最近反馈时间判断在线，确保 LED 和安全逻辑及时响应。 */
    g_gm6020_online =
        (g_gm6020.is_online &&
         (App_ElapsedMs(g_ms, g_gm6020.last_rx_ms) <=
          g_gm6020.online_timeout_ms)) ? 1U : 0U;
    g_vesc_online =
        (g_vesc.is_online &&
         (App_ElapsedMs(g_ms, g_vesc.last_rx_ms) <=
          g_vesc.online_timeout_ms)) ? 1U : 0U;

    if (g_ms < APP_SELF_TEST_MS)
    {
        /* 自检期间禁止整个舵轮使能，而不只是点亮 LED。 */
        App_DisableWheel();
        GM6020_ControlStep(&g_gm6020, g_ms);
        VESC_ControlStep(&g_vesc, g_ms);
        return;
    }

    master_ready =
        (g_master.link_valid && g_master.control_seen) ? 1U : 0U;
    actuators_online =
        (g_gm6020_online && g_vesc_online) ? 1U : 0U;

    /*
     * 故障闭合：
     * - master_ready 要求心跳链路有效且已收到控制目标；
     * - actuators_online 要求两个执行器都在超时窗口内。
     * 清除 pending 可防止故障期间收到的控制帧在恢复后被误执行。
     */
    if (!master_ready || !actuators_online)
    {
        g_control_pending = 0U;
        App_DisableWheel();
    }
    else
    {
        if (g_control_pending)
        {
            /* 新控制帧只在这里应用一次。 */
            App_ApplyPendingControl();
        }

        if (g_wheel_active && !g_need_fresh_control)
        {
            /* GM6020 使用位置斜坡；VESC ERPM 按主控命令直接输出。 */
            SetpointRamp_Update(&g_target_ramp, 0.001f);
            GM6020_SetTargetAngle(
                &g_gm6020,
                SetpointRamp_GetOutput(&g_target_ramp));
        }
    }

    GM6020_ControlStep(&g_gm6020, g_ms);
    VESC_ControlStep(&g_vesc, g_ms);
}

/**
 * @brief 组装 UART 遥测中的故障标志位。
 * @note 这些标志只用于诊断；控制安全由 App_ControlStep 的实时条件直接决定。
 */
static uint32_t App_GetFaultFlags(void)
{
    uint32_t flags = 0U;

    if (!g_master.link_valid)
    {
        flags |= APP_FAULT_MASTER_LINK;
    }
    if (!g_master.control_seen)
    {
        flags |= APP_FAULT_NO_CONTROL;
    }
    if (!g_gm6020_online)
    {
        flags |= APP_FAULT_GM6020_OFFLINE;
    }
    if (!g_vesc_online)
    {
        flags |= APP_FAULT_VESC_OFFLINE;
    }
    if (g_need_fresh_control)
    {
        flags |= APP_FAULT_NEED_FRESH;
    }
    if (g_gm6020.last_tx_status != HAL_OK)
    {
        flags |= APP_FAULT_GM6020_TX;
    }
    if (g_vesc.last_tx_status != HAL_OK)
    {
        flags |= APP_FAULT_VESC_TX;
    }

    return flags;
}

/**
 * @brief 按在线、使能和整轮活动状态刷新四个 LED。
 * @note LED 为低电平点亮；500 ms 自检阶段四灯全亮。
 */
static void App_UpdateLeds(void)
{
    uint8_t enabled;

    if (g_ms < APP_SELF_TEST_MS)
    {
        HAL_GPIO_WritePin(
            GPIOB,
            APP_LED_MASTER_PIN |
                APP_LED_VESC_PIN |
                APP_LED_ENABLE_PIN |
                APP_LED_GM6020_PIN,
            GPIO_PIN_RESET);
        return;
    }

    enabled =
        (g_wheel_active &&
         g_gm6020.is_enabled &&
         g_vesc.is_enabled) ? 1U : 0U;

    HAL_GPIO_WritePin(GPIOB,
                      APP_LED_MASTER_PIN,
                      g_master.link_valid ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,
                      APP_LED_VESC_PIN,
                      g_vesc_online ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,
                      APP_LED_ENABLE_PIN,
                      enabled ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,
                      APP_LED_GM6020_PIN,
                      g_gm6020_online ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
 * @brief 以 50 Hz 组装并通过 UART1 中断发送一帧 CSV 遥测。
 *
 * 通道顺序（角度类均为实际值乘 100）：
 *   1 时间ms, 2 主控链路, 3 控制可用, 4 GM在线, 5 VESC在线,
 *   6 整轮活动, 7 方向反转, 8 需新控制, 9 主控ERPM, 10 角度码,
 *   11 主控目标角, 12 映射后目标角, 13 最终舵向角, 14 最终ERPM,
 *   15 GM实际角, 16 GM转速, 17 GM电流, 18 GM温度, 19 GM目标电压,
 *   20 VESC实际ERPM, 21 VESC电流mA, 22 VESC占空比×1000,
 *   23 斜坡输出角, 24 斜坡速度, 25 控制帧年龄ms,
 *   26 心跳帧年龄ms, 27 故障标志。
 *
 * @note UART 忙时跳过本次遥测，绝不阻塞 1 kHz 控制。
 */
static void App_SendTelemetry(void)
{
    int length;

    if (g_uart_busy)
    {
        return;
    }

    length = snprintf(
        g_uart_buffer,
        sizeof(g_uart_buffer),
        "%u,%u,%u,%u,%u,%u,%u,%u,"
        "%d,%u,%d,%d,%d,%d,"
        "%d,%d,%d,%u,%d,"
        "%d,%d,%d,%d,%d,"
        "%u,%u,%u\r\n",
        (unsigned)g_ms,
        (unsigned)g_master.link_valid,
        (unsigned)g_master.control_seen,
        (unsigned)g_gm6020_online,
        (unsigned)g_vesc_online,
        (unsigned)g_wheel_active,
        (unsigned)g_angle_reversed,
        (unsigned)g_need_fresh_control,
        (int)g_master.drive_erpm,
        (unsigned)g_master.angle_code,
        (int)(g_master.target_angle_deg * 100.0f),
        (int)(g_mapped_target_deg * 100.0f),
        (int)(g_final_target_angle_deg * 100.0f),
        (int)g_final_target_erpm,
        (int)(g_gm6020.angle_deg * 100.0f),
        (int)g_gm6020.speed_rpm,
        (int)g_gm6020.current_raw,
        (unsigned)g_gm6020.temperature,
        (int)g_gm6020.target_voltage,
        (int)g_vesc.actual_erpm,
        (int)(g_vesc.actual_current_a * 1000.0f),
        (int)(g_vesc.duty * 1000.0f),
        (int)(SetpointRamp_GetOutput(&g_target_ramp) * 100.0f),
        (int)SetpointRamp_GetVelocity(&g_target_ramp),
        (unsigned)App_AgeMs(g_ms, g_master.last_control_ms),
        (unsigned)App_AgeMs(g_ms, g_master.last_heartbeat_ms),
        (unsigned)App_GetFaultFlags());

    if ((length > 0) &&
        ((uint32_t)length < sizeof(g_uart_buffer)))
    {
        if (HAL_UART_Transmit_IT(
                &huart1,
                (uint8_t *)g_uart_buffer,
                (uint16_t)length) == HAL_OK)
        {
            g_uart_busy = 1U;
        }
    }
}

/**
 * @brief 初始化应用上下文、驱动对象和串口中断。
 * @note CAN 总线和 TIM6 在本函数之后由 main 按顺序启动。
 */
static void App_Init(void)
{
    g_ms = 0U;
    g_control_pending = 0U;
    g_gm6020_online = 0U;
    g_vesc_online = 0U;
    g_wheel_active = 0U;
    g_need_fresh_control = 1U;
    g_angle_reversed = 0U;
    g_mapped_target_deg = 0.0f;
    g_final_target_angle_deg = 0.0f;
    g_final_target_erpm = 0;
    g_uart_busy = 0U;
    g_last_telemetry_ms = 0U;
    g_last_led_ms = 0U;

    MasterCan_Init(&g_master, APP_MASTER_WHEEL_ID);

    GM6020_Init(&g_gm6020,
                &hcan1,
                APP_GM6020_ID,
                APP_GM6020_ZERO_RAW,
                1,
                1,
                1.0f);

    VESC_Init(&g_vesc, &hcan1, APP_VESC_CONTROLLER_ID);

    SetpointRamp_Init(
        &g_target_ramp,
        0.0f,
        SETPOINT_RAMP_DEFAULT_MAX_SPEED_DEG_S,
        SETPOINT_RAMP_DEFAULT_MAX_ACCEL_DEG_S2);

    HAL_NVIC_SetPriority(USART1_IRQn, 3U, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* 先建立应用对象，再启动接收总线和 1 kHz 控制节拍。 */
  App_Init();

  /* CAN1 承载 GM6020 和 VESC，对本测试场景接收全部帧类型。 */
  if (BSP_CAN_StartAcceptAll(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  /* CAN2 只接收本轮的 0x101 控制帧和 0x111 心跳帧。 */
  if (BSP_CAN_StartMasterFilter(
          &hcan2,
          APP_MASTER_WHEEL_ID) != HAL_OK)
  {
    Error_Handler();
  }

  /* 模块和 CAN 均就绪后最后启动控制中断。 */
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 主循环只做低频状态显示和遥测，时间基准来自 TIM6。 */
    if ((uint32_t)(g_ms - g_last_led_ms) >= APP_LED_UPDATE_MS)
    {
      g_last_led_ms = g_ms;
      App_UpdateLeds();
    }

    if ((uint32_t)(g_ms - g_last_telemetry_ms) >= APP_TELEMETRY_MS)
    {
      g_last_telemetry_ms = g_ms;
      App_SendTelemetry();
    }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief CAN1/CAN2 共用的 RX FIFO0 中断入口。
 *
 * CAN1 帧同时交给 GM6020 和 VESC 解码，两个驱动会自行过滤无关 ID。
 * CAN2 帧交给 MasterCan；只有格式正确的本轮控制帧才置 control_pending。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    uint8_t valid_master_control;

    if ((hcan != &hcan1) && (hcan != &hcan2))
    {
        return;
    }

    while (HAL_CAN_GetRxFifoFillLevel(
               hcan,
               CAN_RX_FIFO0) > 0U)
    {
        if (HAL_CAN_GetRxMessage(
                hcan,
                CAN_RX_FIFO0,
                &header,
                data) != HAL_OK)
        {
            break;
        }

        if (hcan == &hcan1)
        {
            GM6020_OnRx(&g_gm6020, &header, data, g_ms);
            VESC_OnRx(&g_vesc, &header, data, g_ms);
        }
        else
        {
            valid_master_control =
                ((header.IDE == CAN_ID_STD) &&
                 (header.RTR == CAN_RTR_DATA) &&
                 (header.DLC == 8U) &&
                 (header.StdId == g_master.control_id)) ? 1U : 0U;

            MasterCan_OnRx(&g_master, &header, data, g_ms);

            if (valid_master_control)
            {
                g_control_pending = 1U;
            }
        }
    }
}

/**
 * @brief TIM6 1 kHz 周期回调，驱动整个舵轮的实时控制。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if ((htim != NULL) && (htim->Instance == TIM6))
    {
        g_ms++;
        App_ControlStep();
    }
}

/**
 * @brief USART1 中断入口，由 HAL 完成非阻塞发送中断处理。
 */
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

/**
 * @brief UART1 发送完成回调，释放下一次遥测发送许可。
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        g_uart_busy = 0U;
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
