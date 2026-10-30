#include <math.h>
#include <stdio.h>
#include <string.h>

#include "GM6020.h"
#include "bsp_can.h"

const int GM6020_STUB_CAN1 = 1;
const int GM6020_STUB_CAN2 = 2;

static CAN_TxHeaderTypeDef g_last_tx_header;
static uint8_t g_last_tx_data[8];
static uint32_t g_tx_count;
static CAN_FilterTypeDef g_filters[4];
static uint32_t g_filter_count;
static uint32_t g_start_count;
static uint32_t g_notify_count;
static char g_call_order[16];
static uint32_t g_call_order_len;

static uint32_t g_failures;

static void Test_RecordCall(char call)
{
    if (g_call_order_len < (sizeof(g_call_order) - 1U))
    {
        g_call_order[g_call_order_len++] = call;
        g_call_order[g_call_order_len] = '\0';
    }
}

void Error_Handler(void)
{
    g_failures++;
}

HAL_StatusTypeDef HAL_CAN_ConfigFilter(
    CAN_HandleTypeDef *hcan,
    CAN_FilterTypeDef *filter)
{
    (void)hcan;

    Test_RecordCall('F');

    if (g_filter_count < 4U)
    {
        g_filters[g_filter_count] = *filter;
    }
    g_filter_count++;

    return HAL_OK;
}

HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *hcan)
{
    (void)hcan;
    Test_RecordCall('S');
    g_start_count++;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_CAN_ActivateNotification(
    CAN_HandleTypeDef *hcan,
    uint32_t notification)
{
    (void)hcan;
    (void)notification;
    Test_RecordCall('N');
    g_notify_count++;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_CAN_AddTxMessage(
    CAN_HandleTypeDef *hcan,
    CAN_TxHeaderTypeDef *header,
    uint8_t data[8],
    uint32_t *mailbox)
{
    (void)hcan;

    g_last_tx_header = *header;
    memcpy(g_last_tx_data, data, 8U);
    g_tx_count++;

    if (mailbox != NULL)
    {
        *mailbox = 0U;
    }

    return HAL_OK;
}

static void Check(int condition, const char *message)
{
    if (!condition)
    {
        printf("FAIL: %s\n", message);
        g_failures++;
    }
}

static void Test_Wrap180(void)
{
    Check(fabsf(GM6020_Wrap180(359.0f - 1.0f) + 2.0f) < 0.0001f,
          "Wrap180 359->1");
    Check(fabsf(GM6020_Wrap180(1.0f - 359.0f) - 2.0f) < 0.0001f,
          "Wrap180 1->359");
    Check(fabsf(GM6020_Wrap180(180.0f) + 180.0f) < 0.0001f,
          "Wrap180 +180");
    Check(fabsf(GM6020_Wrap180(-180.0f) + 180.0f) < 0.0001f,
          "Wrap180 -180");
}

static void Test_FeedbackAndTimeout(void)
{
    GM6020_t motor;
    CAN_HandleTypeDef can1 = {CAN1};
    CAN_RxHeaderTypeDef header = {0};
    uint8_t data[8] =
    {
        0x0B, 0xE8,
        0x00, 0x64,
        0x00, 0x7B,
        25U, 0U
    };

    GM6020_Init(&motor, &can1, 1U, 1000U, 1, 1, 1.0f);

    header.StdId = 0x205U;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;

    GM6020_OnRx(&motor, &header, data, 100U);

    Check(motor.raw_angle == 3048U, "raw angle decode");
    Check(fabsf(motor.angle_deg - 90.0f) < 0.01f,
          "zero/sign angle conversion");
    Check(motor.speed_rpm == 100, "speed decode");
    Check(motor.current_raw == 123, "current decode");
    Check(motor.temperature == 25U, "temperature decode");
    Check(motor.is_online == 1U, "online after feedback");
    Check(fabsf(motor.target_angle_deg - 90.0f) < 0.01f,
          "first feedback seeds target");

    GM6020_Enable(&motor);
    GM6020_ControlStep(&motor, 120U);
    Check(motor.is_online == 1U, "20 ms is still online");

    GM6020_ControlStep(&motor, 121U);
    Check(motor.is_online == 0U, "21 ms is offline");
    Check(motor.target_voltage == 0, "timeout clears target voltage");
    Check((g_last_tx_data[0] == 0U) &&
          (g_last_tx_data[1] == 0U),
          "timeout sends zero voltage");
}

static void Test_VoltagePacking(void)
{
    GM6020_t motor;
    CAN_HandleTypeDef can1 = {CAN1};

    GM6020_Init(&motor, &can1, 1U, 0U, 1, 1, 1.0f);
    Check(GM6020_SendVoltage(&motor, 1234) == HAL_OK,
          "ID1 send");
    Check(g_last_tx_header.StdId == 0x1FFU,
          "ID1 frame id");
    Check((g_last_tx_data[0] == 0x04U) &&
          (g_last_tx_data[1] == 0xD2U),
          "ID1 first voltage slot");
    Check((g_last_tx_data[2] == 0U) &&
          (g_last_tx_data[3] == 0U) &&
          (g_last_tx_data[4] == 0U) &&
          (g_last_tx_data[5] == 0U) &&
          (g_last_tx_data[6] == 0U) &&
          (g_last_tx_data[7] == 0U),
          "ID1 unused slots zero");
    Check(motor.last_tx_status == HAL_OK,
          "TX status recorded");
    Check(motor.tx_ok_count == 1U,
          "TX success counter");
    Check(motor.last_tx_voltage == 1234,
          "last TX voltage recorded");
    Check((motor.last_tx_data[0] == 0x04U) &&
          (motor.last_tx_data[1] == 0xD2U),
          "last TX data recorded");

    GM6020_Init(&motor, &can1, 5U, 0U, 1, 1, 1.0f);
    Check(GM6020_SendVoltage(&motor, 1234) == HAL_OK,
          "ID5 send");
    Check(g_last_tx_header.StdId == 0x2FFU,
          "ID5 frame id");
    Check((g_last_tx_data[0] == 0x04U) &&
          (g_last_tx_data[1] == 0xD2U),
          "ID5 first voltage slot");
    Check(g_last_tx_data[2] == 0U, "ID5 unused slot zero");
}

static void Test_ControlIntegralPolicy(void)
{
    GM6020_t motor;
    CAN_HandleTypeDef can1 = {CAN1};
    float initial_integral;
    int16_t initial_voltage;
    uint32_t i;

    GM6020_Init(&motor, &can1, 1U, 0U, 1, 1, 1.0f);

    Check(motor.position_pid.integral_separation == 1U,
          "position integral separation remains enabled");
    Check(motor.speed_pid.integral_separation == 0U,
          "speed integral separation disabled in driver");

    motor.is_online = 1U;
    motor.is_enabled = 1U;
    motor.voltage_limit_raw = 5000;
    motor.max_speed_rpm = 300.0f;
    motor.target_angle_deg = 15.0f;
    motor.angle_deg = 0.0f;
    motor.speed_rpm = 0;

    for (i = 0U; i < 10U; i++)
    {
        motor.last_rx_ms = i;
        GM6020_ControlStep(&motor, i);
    }

    Check(fabsf(motor.position_pid.output_max - 300.0f) < 0.001f,
          "position anti-windup limit follows max speed");
    Check(fabsf(motor.speed_pid.output_max - 5000.0f) < 0.001f,
          "speed anti-windup limit follows voltage limit");
    Check(fabsf(motor.speed_pid.integral) < 0.001f,
          "speed integral does not wind beyond active limit");
    Check(motor.target_voltage == 5000,
          "speed output is limited to active voltage limit");

    GM6020_Init(&motor, &can1, 1U, 0U, 1, 1, 1.0f);
    motor.is_online = 1U;
    motor.is_enabled = 1U;
    motor.target_angle_deg = 4.5f;
    motor.angle_deg = 0.0f;
    motor.speed_rpm = 0;
    motor.position_pid.kd = 0.0f;
    motor.position_pid.integral = 50.0f;
    motor.speed_pid.integral = -4800.0f;
    initial_integral = motor.speed_pid.integral;

    motor.last_rx_ms = 0U;
    GM6020_ControlStep(&motor, 0U);
    initial_voltage = motor.target_voltage;

    for (i = 1U; i < 100U; i++)
    {
        motor.last_rx_ms = i;
        GM6020_ControlStep(&motor, i);
    }

    Check(motor.speed_pid.integral > initial_integral + 500.0f,
          "speed integral recovers from negative bias at standstill");
    Check(motor.target_voltage > initial_voltage + 200,
          "voltage continues increasing at standstill");
}

static void Test_BspCan(void)
{
    CAN_HandleTypeDef can1 = {CAN1};
    CAN_HandleTypeDef can2 = {CAN2};

    g_filter_count = 0U;
    g_start_count = 0U;
    g_notify_count = 0U;
    g_call_order_len = 0U;
    g_call_order[0] = '\0';

    Check(BSP_CAN_StartAcceptAll(&can1) == HAL_OK,
          "accept-all start");
    Check(g_filter_count == 1U, "accept-all filter count");
    Check(g_filters[0].FilterBank == 0U,
          "CAN1 accept-all bank");
    Check((g_filters[0].FilterIdHigh == 0U) &&
          (g_filters[0].FilterMaskIdHigh == 0U) &&
          (g_filters[0].FilterMaskIdLow == 0U),
          "accept-all mask");
    Check(strcmp(g_call_order, "FSN") == 0,
          "filter/start/notify order");

    Check(BSP_CAN_StartMasterFilter(&can2, 1U) == HAL_OK,
          "master filter start");
    Check(g_filter_count == 3U, "master filter count");
    Check(g_filters[1].FilterBank == 14U,
          "control filter bank");
    Check(g_filters[2].FilterBank == 15U,
          "heartbeat filter bank");
    Check(g_filters[1].FilterIdHigh == (uint32_t)(0x101U << 5U),
          "control standard id");
    Check(g_filters[2].FilterIdHigh == (uint32_t)(0x111U << 5U),
          "heartbeat standard id");
    Check(strcmp(g_call_order, "FSNFFSN") == 0,
          "master filter order");
    Check((g_start_count == 2U) && (g_notify_count == 2U),
          "start and notify called once per bus");
}

int main(void)
{
    Test_Wrap180();
    Test_FeedbackAndTimeout();
    Test_VoltagePacking();
    Test_ControlIntegralPolicy();
    Test_BspCan();

    if (g_failures != 0U)
    {
        printf("GM6020 host tests: %u failure(s)\n",
               (unsigned)g_failures);
        return 1;
    }

    printf("GM6020 host tests: all passed\n");
    return 0;
}
