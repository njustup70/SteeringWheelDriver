#include "vesc_test.h"

#include <stdio.h>

#include "bsp_can.h"
#include "tim.h"
#include "usart.h"

#define VESC_TEST_LED_HEARTBEAT_PIN  GPIO_PIN_4
#define VESC_TEST_LED_ONLINE_PIN     GPIO_PIN_5
#define VESC_TEST_LED_ENABLE_PIN     GPIO_PIN_6

#define VESC_TEST_SELF_TEST_MS       500U
#define VESC_TEST_TELEMETRY_MS       20U

volatile VESCTestMode_t g_vesc_test_mode = VESC_TEST_DISABLED;
volatile uint8_t g_vesc_test_enable = 0U;
volatile int32_t g_vesc_test_target_erpm = 0;
volatile int32_t g_vesc_test_erpm_limit =
    VESC_TEST_DEFAULT_ERPM_LIMIT;
volatile uint32_t g_vesc_test_ms = 0U;
volatile uint32_t g_can1_error_code = 0U;
volatile uint32_t g_can1_esr = 0U;
volatile uint32_t g_can1_tsr = 0U;
volatile uint32_t g_can1_rf0r = 0U;
volatile uint32_t g_can1_state = 0U;
volatile uint32_t g_can1_tx_free = 0U;

VESC_t g_vesc_motor;

static CAN_HandleTypeDef *g_can;
static UART_HandleTypeDef *g_uart;
static volatile uint8_t g_uart_busy;
static char g_uart_buffer[384];
static uint32_t g_last_telemetry_ms;
static uint8_t g_self_test_active;
static VESCTestMode_t g_last_mode;

static void Test_LedWrite(uint16_t pin, uint8_t on)
{
    HAL_GPIO_WritePin(GPIOB, pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void Test_LedAll(uint8_t on)
{
    Test_LedWrite(VESC_TEST_LED_HEARTBEAT_PIN, on);
    Test_LedWrite(VESC_TEST_LED_ONLINE_PIN, on);
    Test_LedWrite(VESC_TEST_LED_ENABLE_PIN, on);
    Test_LedWrite(GPIO_PIN_7, on);
}

static void Test_UpdateLeds(void)
{
    uint8_t heartbeat =
        (((g_vesc_test_ms / 500U) % 2U) == 0U) ? 1U : 0U;

    Test_LedWrite(VESC_TEST_LED_HEARTBEAT_PIN, heartbeat);
    Test_LedWrite(VESC_TEST_LED_ONLINE_PIN,
                  g_vesc_motor.is_online);
    Test_LedWrite(VESC_TEST_LED_ENABLE_PIN,
                  g_vesc_motor.is_enabled);
    Test_LedWrite(GPIO_PIN_7, 0U);
}

static int32_t Test_ErpmLimit(void)
{
    int32_t limit = g_vesc_test_erpm_limit;

    return (limit > 0) ? limit : 0;
}

static int32_t Test_ClampErpm(int32_t erpm, int32_t limit)
{
    if (erpm > limit)
    {
        return limit;
    }

    if (erpm < -limit)
    {
        return -limit;
    }

    return erpm;
}

static void Test_DisableIfEnabled(void)
{
    if (g_vesc_motor.is_enabled)
    {
        VESC_Disable(&g_vesc_motor);
    }
}

static void Test_UpdateTimeout(void)
{
    if (g_vesc_motor.is_online &&
        ((uint32_t)(g_vesc_test_ms - g_vesc_motor.last_rx_ms) >
         g_vesc_motor.online_timeout_ms))
    {
        g_vesc_test_enable = 0U;
        Test_DisableIfEnabled();
    }
}

static void Test_UpdateCommand(void)
{
    int32_t command_erpm = 0;
    int32_t limit_erpm = Test_ErpmLimit();

    if (!g_vesc_test_enable || !g_vesc_motor.is_online)
    {
        VESC_SetTargetErpm(&g_vesc_motor, 0);
        return;
    }

    if (g_vesc_test_mode == VESC_TEST_STEP)
    {
        command_erpm =
            Test_ClampErpm(g_vesc_test_target_erpm, limit_erpm);
    }

    VESC_SetTargetErpm(&g_vesc_motor, command_erpm);
}

static void Test_UpdateMode(void)
{
    if (g_vesc_test_mode != g_last_mode)
    {
        g_last_mode = g_vesc_test_mode;
        g_vesc_test_enable = 0U;
        VESC_Disable(&g_vesc_motor);
    }

    switch (g_vesc_test_mode)
    {
        case VESC_TEST_DISABLED:
            Test_DisableIfEnabled();
            break;

        case VESC_TEST_STEP:
            if (!g_vesc_motor.is_online)
            {
                g_vesc_test_enable = 0U;
                Test_DisableIfEnabled();
            }
            else if (!g_vesc_test_enable)
            {
                Test_DisableIfEnabled();
            }
            else if (!g_vesc_motor.is_enabled)
            {
                VESC_Enable(&g_vesc_motor);
            }
            break;

        default:
            g_vesc_test_mode = VESC_TEST_DISABLED;
            g_vesc_test_enable = 0U;
            VESC_Disable(&g_vesc_motor);
            break;
    }
}

static void Test_SendTelemetry(void)
{
    int actual_current_ma;
    int duty_milli;
    int last_tx_status;
    int last_tx_erpm;
    int last_tx_data0;
    int last_tx_data1;
    int length;
    uint32_t rx_age_ms;

    if ((g_uart == NULL) || g_uart_busy)
    {
        return;
    }

    actual_current_ma =
        (int)(g_vesc_motor.actual_current_a * 1000.0f);
    duty_milli = (int)(g_vesc_motor.duty * 1000.0f);
    last_tx_status = (int)g_vesc_motor.last_tx_status;
    last_tx_erpm = (int)g_vesc_motor.last_tx_erpm;
    last_tx_data0 = (int)g_vesc_motor.last_tx_data[0];
    last_tx_data1 = (int)g_vesc_motor.last_tx_data[1];
    rx_age_ms = (g_vesc_motor.last_rx_ms == 0U)
                    ? 0xFFFFFFFFU
                    : (uint32_t)(g_vesc_test_ms -
                                 g_vesc_motor.last_rx_ms);

    length = snprintf(
        g_uart_buffer,
        sizeof(g_uart_buffer),
        "%d,%d,%d,%d,%u,%u,%u,%u,"
        "%d,%d,%u,%u,%u,%u,%d,%d,%d,"
        "%u,%u,%u,%u,%u,%u\r\n",
        (int)g_vesc_motor.target_erpm,
        (int)g_vesc_motor.actual_erpm,
        actual_current_ma,
        duty_milli,
        (unsigned)g_vesc_motor.is_online,
        (unsigned)g_vesc_motor.is_enabled,
        (unsigned)g_vesc_test_mode,
        (unsigned)rx_age_ms,
        (int)g_vesc_test_erpm_limit,
        last_tx_status,
        (unsigned)g_vesc_motor.tx_ok_count,
        (unsigned)g_vesc_motor.tx_busy_count,
        (unsigned)g_vesc_motor.tx_error_count,
        (unsigned)g_vesc_motor.last_tx_id,
        last_tx_erpm,
        last_tx_data0,
        last_tx_data1,
        (unsigned)g_can1_error_code,
        (unsigned)g_can1_esr,
        (unsigned)g_can1_tsr,
        (unsigned)g_can1_rf0r,
        (unsigned)g_can1_state,
        (unsigned)g_can1_tx_free);

    if ((length > 0) &&
        ((uint32_t)length < sizeof(g_uart_buffer)))
    {
        if (HAL_UART_Transmit_IT(
                g_uart,
                (uint8_t *)g_uart_buffer,
                (uint16_t)length) == HAL_OK)
        {
            g_uart_busy = 1U;
        }
    }
}

void VESCTest_Init(CAN_HandleTypeDef *hcan,
                   UART_HandleTypeDef *huart)
{
    if ((hcan == NULL) || (huart == NULL))
    {
        Error_Handler();
    }

    g_can = hcan;
    g_uart = huart;
    g_uart_busy = 0U;
    g_last_telemetry_ms = 0U;
    g_vesc_test_mode = VESC_TEST_DISABLED;
    g_vesc_test_enable = 0U;
    g_vesc_test_target_erpm = 0;
    g_vesc_test_erpm_limit = VESC_TEST_DEFAULT_ERPM_LIMIT;
    g_vesc_test_ms = 0U;

    g_last_mode = VESC_TEST_DISABLED;
    g_self_test_active = 1U;

    VESC_Init(&g_vesc_motor, hcan, VESC_TEST_CONTROLLER_ID);

    Test_LedAll(1U);

    HAL_NVIC_SetPriority(USART1_IRQn, 3U, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void VESCTest_Loop(void)
{
    if (g_can != NULL)
    {
        g_can1_error_code = HAL_CAN_GetError(g_can);
        g_can1_state = (uint32_t)HAL_CAN_GetState(g_can);
        g_can1_tx_free =
            HAL_CAN_GetTxMailboxesFreeLevel(g_can);
        g_can1_esr = g_can->Instance->ESR;
        g_can1_tsr = g_can->Instance->TSR;
        g_can1_rf0r = g_can->Instance->RF0R;
    }

    if (g_self_test_active)
    {
        if (g_vesc_test_ms < VESC_TEST_SELF_TEST_MS)
        {
            Test_LedAll(1U);
            return;
        }

        g_self_test_active = 0U;
        Test_LedAll(0U);
    }

    Test_UpdateTimeout();
    Test_UpdateMode();
    Test_UpdateLeds();

    if ((uint32_t)(g_vesc_test_ms - g_last_telemetry_ms) >=
        VESC_TEST_TELEMETRY_MS)
    {
        g_last_telemetry_ms = g_vesc_test_ms;
        Test_SendTelemetry();
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if ((g_can == NULL) || (hcan != g_can))
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

        VESC_OnRx(&g_vesc_motor,
                  &header,
                  data,
                  g_vesc_test_ms);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if ((htim != NULL) && (htim->Instance == TIM6))
    {
        g_vesc_test_ms++;
        Test_UpdateCommand();
        VESC_ControlStep(&g_vesc_motor, g_vesc_test_ms);
    }
}

void USART1_IRQHandler(void)
{
    if (g_uart != NULL)
    {
        HAL_UART_IRQHandler(g_uart);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == g_uart)
    {
        g_uart_busy = 0U;
    }
}
