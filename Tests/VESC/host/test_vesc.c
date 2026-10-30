#include <stdio.h>
#include <string.h>

#include "VESC.h"

const int VESC_STUB_CAN1 = 1;

static CAN_TxHeaderTypeDef g_tx_header;
static uint8_t g_tx_data[8];
static uint32_t g_tx_count;
static HAL_StatusTypeDef g_tx_result = HAL_OK;
static uint32_t g_failures;

void Error_Handler(void)
{
    g_failures++;
}

HAL_StatusTypeDef HAL_CAN_AddTxMessage(
    CAN_HandleTypeDef *hcan,
    CAN_TxHeaderTypeDef *header,
    uint8_t data[8],
    uint32_t *mailbox)
{
    (void)hcan;

    g_tx_header = *header;
    memcpy(g_tx_data, data, sizeof(g_tx_data));
    g_tx_count++;

    if (mailbox != NULL)
    {
        *mailbox = 0U;
    }

    return g_tx_result;
}

static void Check(int condition, const char *message)
{
    if (!condition)
    {
        printf("FAIL: %s\n", message);
        g_failures++;
    }
}

static void EncodeInt32(uint8_t *data, int32_t value)
{
    uint32_t raw = (uint32_t)value;

    data[0] = (uint8_t)(raw >> 24U);
    data[1] = (uint8_t)(raw >> 16U);
    data[2] = (uint8_t)(raw >> 8U);
    data[3] = (uint8_t)raw;
}

static void EncodeInt16(uint8_t *data, int16_t value)
{
    uint16_t raw = (uint16_t)value;

    data[0] = (uint8_t)(raw >> 8U);
    data[1] = (uint8_t)raw;
}

static void MakeStatusHeader(CAN_RxHeaderTypeDef *header)
{
    memset(header, 0, sizeof(*header));
    header->ExtId = (uint32_t)(VESC_PACKET_STATUS << 8U) | 67U;
    header->IDE = CAN_ID_EXT;
    header->RTR = CAN_RTR_DATA;
    header->DLC = 8U;
}

static void Test_StatusDecode(void)
{
    VESC_t vesc;
    CAN_HandleTypeDef can1 = {&VESC_STUB_CAN1};
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    VESC_Init(&vesc, &can1, 67U);
    MakeStatusHeader(&header);
    EncodeInt32(&data[0], -123456);
    EncodeInt16(&data[4], -123);
    EncodeInt16(&data[6], 750);

    VESC_OnRx(&vesc, &header, data, 1000U);

    Check(vesc.actual_erpm == -123456, "signed ERPM decode");
    Check(vesc.actual_current_a < -12.29f &&
          vesc.actual_current_a > -12.31f,
          "current decode");
    Check(vesc.duty > 0.749f && vesc.duty < 0.751f,
          "duty decode");
    Check(vesc.is_online == 1U, "valid status marks online");
    Check(vesc.last_rx_ms == 1000U, "status timestamp");

    header.ExtId = (uint32_t)(VESC_PACKET_STATUS << 8U) | 68U;
    VESC_OnRx(&vesc, &header, data, 1010U);
    Check(vesc.last_rx_ms == 1000U, "wrong controller ID ignored");

    MakeStatusHeader(&header);
    header.IDE = CAN_ID_STD;
    VESC_OnRx(&vesc, &header, data, 1020U);
    Check(vesc.last_rx_ms == 1000U, "standard frame ignored");

    MakeStatusHeader(&header);
    header.DLC = 7U;
    VESC_OnRx(&vesc, &header, data, 1030U);
    Check(vesc.last_rx_ms == 1000U, "wrong DLC ignored");

    MakeStatusHeader(&header);
    header.RTR = CAN_RTR_REMOTE;
    VESC_OnRx(&vesc, &header, data, 1040U);
    Check(vesc.last_rx_ms == 1000U, "remote frame ignored");
}

static void Test_TimeoutBoundary(void)
{
    VESC_t vesc;
    CAN_HandleTypeDef can1 = {&VESC_STUB_CAN1};
    CAN_RxHeaderTypeDef header;
    uint8_t data[8] = {0};

    VESC_Init(&vesc, &can1, 67U);
    MakeStatusHeader(&header);
    VESC_OnRx(&vesc, &header, data, 0U);
    VESC_Enable(&vesc);
    VESC_SetTargetErpm(&vesc, -321);

    VESC_ControlStep(&vesc, 100U);
    Check(vesc.is_online == 1U, "100 ms remains online");
    Check(g_tx_data[0] == 0xFFU && g_tx_data[1] == 0xFFU &&
          g_tx_data[2] == 0xFEU && g_tx_data[3] == 0xBFU,
          "enabled target sent via SET_RPM");

    VESC_ControlStep(&vesc, 101U);
    Check(vesc.is_online == 0U, "101 ms marks offline");
    Check(g_tx_data[0] == 0U && g_tx_data[1] == 0U &&
          g_tx_data[2] == 0U && g_tx_data[3] == 0U,
          "offline sends zero ERPM");
}

static void Test_TxPacket(void)
{
    VESC_t vesc;
    CAN_HandleTypeDef can1 = {&VESC_STUB_CAN1};
    CAN_RxHeaderTypeDef status;
    uint8_t data[8] = {0};
    uint32_t before;

    VESC_Init(&vesc, &can1, 67U);
    MakeStatusHeader(&status);
    VESC_OnRx(&vesc, &status, data, 1U);
    VESC_Enable(&vesc);
    VESC_SetTargetErpm(&vesc, 1234);

    before = g_tx_count;
    Check(VESC_SendErpm(&vesc, 1234) == HAL_OK,
          "SET_RPM send succeeds");
    Check(g_tx_count == before + 1U, "one CAN frame transmitted");
    Check(g_tx_header.ExtId == 0x343U, "ID 67 SET_RPM extended ID");
    Check(g_tx_header.StdId == 0U, "extended frame clears StdId");
    Check(g_tx_header.IDE == CAN_ID_EXT, "SET_RPM uses extended frame");
    Check(g_tx_header.DLC == 8U, "SET_RPM DLC is 8");
    Check(g_tx_data[0] == 0U && g_tx_data[1] == 0U &&
          g_tx_data[2] == 0x04U && g_tx_data[3] == 0xD2U,
          "SET_RPM big-endian ERPM");
    Check(g_tx_data[4] == 0U && g_tx_data[5] == 0U &&
          g_tx_data[6] == 0U && g_tx_data[7] == 0U,
          "SET_RPM tail bytes zero");
    Check(vesc.last_tx_id == 0x343U, "last TX ID recorded");
    Check(vesc.last_tx_erpm == 1234, "last TX ERPM recorded");
}

static void Test_DisableAndCounters(void)
{
    VESC_t vesc;
    CAN_HandleTypeDef can1 = {&VESC_STUB_CAN1};
    uint32_t before;

    VESC_Init(&vesc, &can1, 67U);
    g_tx_result = HAL_BUSY;
    before = vesc.tx_busy_count;
    Check(VESC_SendErpm(&vesc, 100) == HAL_BUSY,
          "busy status returned");
    Check(vesc.tx_busy_count == before + 1U,
          "busy counter increments");

    g_tx_result = HAL_OK;
    VESC_Enable(&vesc);
    VESC_SetTargetErpm(&vesc, 500);
    VESC_Disable(&vesc);
    Check(vesc.is_enabled == 0U, "disable clears enabled");
    Check(vesc.target_erpm == 0, "disable clears target");
    Check(g_tx_data[0] == 0U && g_tx_data[1] == 0U &&
          g_tx_data[2] == 0U && g_tx_data[3] == 0U,
          "disable sends zero ERPM");
}

int main(void)
{
    Test_StatusDecode();
    Test_TimeoutBoundary();
    Test_TxPacket();
    Test_DisableAndCounters();

    if (g_failures != 0U)
    {
        printf("VESC host tests: %u failure(s)\n", g_failures);
        return 1;
    }

    printf("VESC host tests: all passed\n");
    return 0;
}
