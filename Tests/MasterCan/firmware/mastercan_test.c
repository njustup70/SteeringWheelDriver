#include "mastercan_test.h"

#include <stdio.h>

#include "tim.h"
#include "usart.h"

#define MASTER_CAN_TEST_LED_HEARTBEAT_PIN  GPIO_PIN_4
#define MASTER_CAN_TEST_LED_LINK_PIN       GPIO_PIN_5
#define MASTER_CAN_TEST_LED_CONTROL_PIN    GPIO_PIN_6
#define MASTER_CAN_TEST_LED_RX_PIN         GPIO_PIN_7

#define MASTER_CAN_TEST_SELF_TEST_MS       500U
#define MASTER_CAN_TEST_TELEMETRY_MS       20U
#define MASTER_CAN_TEST_CONTROL_FLASH_MS   100U

MasterCan_t g_master_can;
volatile uint32_t g_master_can_test_ms = 0U;
volatile uint32_t g_mastercan_control_rx_count = 0U;
volatile uint32_t g_mastercan_heartbeat_rx_count = 0U;
volatile uint32_t g_mastercan_invalid_rx_count = 0U;
volatile uint32_t g_can2_error_code = 0U;
volatile uint32_t g_can2_esr = 0U;
volatile uint32_t g_can2_tsr = 0U;
volatile uint32_t g_can2_rf0r = 0U;
volatile uint32_t g_can2_state = 0U;
volatile uint32_t g_can2_rx_fill = 0U;

static CAN_HandleTypeDef *g_can;
static UART_HandleTypeDef *g_uart;
static volatile uint8_t g_uart_busy;
static char g_uart_buffer[320];
static uint32_t g_last_telemetry_ms;
static uint32_t g_control_flash_until_ms;
static uint8_t g_self_test_active;

static void Test_LedWrite(uint16_t pin, uint8_t on)
{
    HAL_GPIO_WritePin(GPIOB, pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void Test_LedAll(uint8_t on)
{
    Test_LedWrite(MASTER_CAN_TEST_LED_HEARTBEAT_PIN, on);
    Test_LedWrite(MASTER_CAN_TEST_LED_LINK_PIN, on);
    Test_LedWrite(MASTER_CAN_TEST_LED_CONTROL_PIN, on);
    Test_LedWrite(MASTER_CAN_TEST_LED_RX_PIN, on);
}

static void Test_UpdateLeds(void)
{
    uint8_t heartbeat =
        (((g_master_can_test_ms / 500U) % 2U) == 0U) ? 1U : 0U;
    uint8_t control_flash =
        ((int32_t)(g_control_flash_until_ms -
                   g_master_can_test_ms) > 0) ? 1U : 0U;

    Test_LedWrite(MASTER_CAN_TEST_LED_HEARTBEAT_PIN, heartbeat);
    Test_LedWrite(MASTER_CAN_TEST_LED_LINK_PIN,
                  g_master_can.link_valid);
    Test_LedWrite(MASTER_CAN_TEST_LED_CONTROL_PIN,
                  g_master_can.control_seen);
    Test_LedWrite(MASTER_CAN_TEST_LED_RX_PIN, control_flash);
}

static uint32_t Test_AgeMs(uint32_t timestamp_ms, uint8_t seen)
{
    if (!seen)
    {
        return 0xFFFFFFFFU;
    }

    return (uint32_t)(g_master_can_test_ms - timestamp_ms);
}

static void Test_SendTelemetry(void)
{
    int length;

    if ((g_uart == NULL) || g_uart_busy)
    {
        return;
    }

    length = snprintf(
        g_uart_buffer,
        sizeof(g_uart_buffer),
        "%u,%u,%u,%u,%d,%u,%d,%u,%u,%u,%u,"
        "%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
        (unsigned)g_master_can_test_ms,
        (unsigned)g_master_can.wheel_id,
        (unsigned)g_master_can.control_id,
        (unsigned)g_master_can.heartbeat_id,
        (int)g_master_can.drive_erpm,
        (unsigned)g_master_can.angle_code,
        (int)(g_master_can.target_angle_deg * 100.0f),
        (unsigned)g_master_can.control_seen,
        (unsigned)g_master_can.link_valid,
        (unsigned)Test_AgeMs(
            g_master_can.last_control_ms,
            (g_mastercan_control_rx_count != 0U) ? 1U : 0U),
        (unsigned)Test_AgeMs(
            g_master_can.last_heartbeat_ms,
            (g_mastercan_heartbeat_rx_count != 0U) ? 1U : 0U),
        (unsigned)g_mastercan_control_rx_count,
        (unsigned)g_mastercan_heartbeat_rx_count,
        (unsigned)g_mastercan_invalid_rx_count,
        (unsigned)g_can2_error_code,
        (unsigned)g_can2_esr,
        (unsigned)g_can2_tsr,
        (unsigned)g_can2_rf0r,
        (unsigned)g_can2_state,
        (unsigned)g_can2_rx_fill);

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

void MasterCanTest_Init(CAN_HandleTypeDef *hcan,
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
    g_control_flash_until_ms = 0U;
    g_self_test_active = 1U;

    g_master_can_test_ms = 0U;
    g_mastercan_control_rx_count = 0U;
    g_mastercan_heartbeat_rx_count = 0U;
    g_mastercan_invalid_rx_count = 0U;
    g_can2_error_code = 0U;
    g_can2_esr = 0U;
    g_can2_tsr = 0U;
    g_can2_rf0r = 0U;
    g_can2_state = 0U;
    g_can2_rx_fill = 0U;

    MasterCan_Init(&g_master_can, MASTER_CAN_TEST_WHEEL_ID);

    Test_LedAll(1U);

    HAL_NVIC_SetPriority(USART1_IRQn, 3U, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void MasterCanTest_Loop(void)
{
    if (g_can != NULL)
    {
        g_can2_error_code = HAL_CAN_GetError(g_can);
        g_can2_state = (uint32_t)HAL_CAN_GetState(g_can);
        g_can2_esr = g_can->Instance->ESR;
        g_can2_tsr = g_can->Instance->TSR;
        g_can2_rf0r = g_can->Instance->RF0R;
        g_can2_rx_fill =
            HAL_CAN_GetRxFifoFillLevel(g_can, CAN_RX_FIFO0);
    }

    if (g_self_test_active)
    {
        if (g_master_can_test_ms < MASTER_CAN_TEST_SELF_TEST_MS)
        {
            Test_LedAll(1U);
            return;
        }

        g_self_test_active = 0U;
        Test_LedAll(0U);
    }

    Test_UpdateLeds();

    if ((uint32_t)(g_master_can_test_ms - g_last_telemetry_ms) >=
        MASTER_CAN_TEST_TELEMETRY_MS)
    {
        g_last_telemetry_ms = g_master_can_test_ms;
        Test_SendTelemetry();
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    uint8_t valid_format;

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

        valid_format =
            ((header.IDE == CAN_ID_STD) &&
             (header.RTR == CAN_RTR_DATA) &&
             (header.DLC == 8U)) ? 1U : 0U;

        if (!valid_format)
        {
            g_mastercan_invalid_rx_count++;
        }
        else if (header.StdId == g_master_can.control_id)
        {
            g_mastercan_control_rx_count++;
            g_control_flash_until_ms =
                g_master_can_test_ms +
                MASTER_CAN_TEST_CONTROL_FLASH_MS;
        }
        else if (header.StdId == g_master_can.heartbeat_id)
        {
            g_mastercan_heartbeat_rx_count++;
        }
        else
        {
            g_mastercan_invalid_rx_count++;
        }

        MasterCan_OnRx(&g_master_can,
                       &header,
                       data,
                       g_master_can_test_ms);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if ((htim != NULL) && (htim->Instance == TIM6))
    {
        g_master_can_test_ms++;
        MasterCan_Update(&g_master_can, g_master_can_test_ms);
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
