#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "MasterCan.h"

static uint32_t g_failures;

static void Check(int condition, const char *message)
{
    if (!condition)
    {
        printf("FAIL: %s\n", message);
        g_failures++;
    }
}

void Error_Handler(void)
{
    g_failures++;
}

static void MakeControlData(uint8_t data[8],
                            int32_t erpm,
                            int32_t angle_code)
{
    uint32_t erpm_raw = (uint32_t)erpm;
    uint32_t angle_raw = (uint32_t)angle_code;

    data[0] = (uint8_t)(erpm_raw >> 24U);
    data[1] = (uint8_t)(erpm_raw >> 16U);
    data[2] = (uint8_t)(erpm_raw >> 8U);
    data[3] = (uint8_t)erpm_raw;

    data[4] = (uint8_t)(angle_raw >> 24U);
    data[5] = (uint8_t)(angle_raw >> 16U);
    data[6] = (uint8_t)(angle_raw >> 8U);
    data[7] = (uint8_t)angle_raw;
}

static CAN_RxHeaderTypeDef MakeHeader(uint32_t std_id)
{
    CAN_RxHeaderTypeDef header = {0};

    header.StdId = std_id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    return header;
}

static void Test_WheelIds(void)
{
    MasterCan_t master;

    MasterCan_Init(&master, 1U);
    Check(master.control_id == 0x101U, "wheel1 control id");
    Check(master.heartbeat_id == 0x111U, "wheel1 heartbeat id");

    MasterCan_Init(&master, 4U);
    Check(master.control_id == 0x104U, "wheel4 control id");
    Check(master.heartbeat_id == 0x114U, "wheel4 heartbeat id");
}

static void Test_Decode(void)
{
    MasterCan_t master;
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    MasterCan_Init(&master, 1U);
    MasterCan_Update(&master, 0U);
    Check(master.control_seen == 0U, "no control before first frame");
    Check(master.link_valid == 0U, "no link before first frame");

    MakeControlData(data, -1234, 8192);
    header = MakeHeader(0x101U);
    MasterCan_OnRx(&master, &header, data, 100U);

    Check(master.drive_erpm == -1234, "signed ERPM decode");
    Check(master.angle_code == 8192U, "angle code decode");
    Check(fabsf(master.target_angle_deg - 180.0f) < 0.001f,
          "angle code to degrees");
    Check(master.control_seen == 1U, "control available immediately");
    Check(master.link_valid == 1U, "control counts as heartbeat");

    MakeControlData(data, 0, 16383);
    MasterCan_OnRx(&master, &header, data, 110U);
    Check(master.angle_code == 16383U, "angle upper bound");

    MakeControlData(data, 0, 16384);
    MasterCan_OnRx(&master, &header, data, 120U);
    Check(master.angle_code == 16383U, "angle over upper bound clamps");

    MakeControlData(data, 0, -1);
    MasterCan_OnRx(&master, &header, data, 130U);
    Check(master.angle_code == 0U, "negative angle clamps to zero");
}

static void Test_InputValidation(void)
{
    MasterCan_t master;
    CAN_RxHeaderTypeDef header;
    uint8_t data[8] = {0};

    MasterCan_Init(&master, 1U);

    header = MakeHeader(0x102U);
    MasterCan_OnRx(&master, &header, data, 100U);
    Check(master.control_seen == 0U, "wrong control id ignored");

    header = MakeHeader(0x101U);
    header.IDE = CAN_ID_EXT;
    MasterCan_OnRx(&master, &header, data, 110U);
    Check(master.control_seen == 0U, "extended frame ignored");

    header = MakeHeader(0x101U);
    header.RTR = 1U;
    MasterCan_OnRx(&master, &header, data, 120U);
    Check(master.control_seen == 0U, "remote frame ignored");

    header = MakeHeader(0x101U);
    header.DLC = 7U;
    MasterCan_OnRx(&master, &header, data, 130U);
    Check(master.control_seen == 0U, "wrong DLC ignored");
}

static void Test_TimeoutAndHeartbeat(void)
{
    MasterCan_t master;
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    MasterCan_Init(&master, 2U);
    MakeControlData(data, 100, 0);
    header = MakeHeader(0x102U);
    MasterCan_OnRx(&master, &header, data, 100U);

    MasterCan_Update(&master, 300U);
    Check(master.control_seen == 1U,
          "control persists without periodic updates");
    Check(master.link_valid == 1U, "control maintains link");

    MasterCan_Update(&master, 600U);
    Check(master.control_seen == 1U, "control valid at link boundary");
    Check(master.link_valid == 1U, "500 ms link still valid");
    MasterCan_Update(&master, 601U);
    Check(master.link_valid == 0U, "501 ms link invalid");
    Check(master.control_seen == 0U,
          "link expiry clears available control");
    Check(master.drive_erpm == 100,
          "link expiry preserves last decoded ERPM");

    MasterCan_Init(&master, 2U);
    header = MakeHeader(0x112U);
    MasterCan_OnRx(&master, &header, data, 100U);
    Check(master.link_valid == 1U, "heartbeat alone keeps link");
    Check(master.control_seen == 0U, "heartbeat does not create control");

    MakeControlData(data, 100, 0);
    header = MakeHeader(0x102U);
    MasterCan_OnRx(&master, &header, data, 100U);
    header = MakeHeader(0x112U);
    MasterCan_OnRx(&master, &header, data, 400U);
    MasterCan_Update(&master, 500U);
    Check(master.control_seen == 1U,
          "heartbeat keeps persistent control available");
    Check(master.link_valid == 1U, "heartbeat refreshes link");

    MasterCan_Update(&master, 900U);
    Check(master.link_valid == 1U,
          "heartbeat valid at 500 ms boundary");

    MasterCan_Update(&master, 901U);
    Check(master.control_seen == 0U,
          "timeout clears control before heartbeat recovery");
    Check(master.link_valid == 0U,
          "heartbeat link expires after 500 ms");

    header = MakeHeader(0x112U);
    MasterCan_OnRx(&master, &header, data, 1000U);
    Check(master.link_valid == 1U, "heartbeat restores link");
    Check(master.control_seen == 0U,
          "heartbeat recovery does not restore old control");

    MakeControlData(data, 200, 4096);
    header = MakeHeader(0x102U);
    MasterCan_OnRx(&master, &header, data, 1100U);
    Check(master.control_seen == 1U,
          "new control restores available command");
    Check(master.drive_erpm == 200, "new control replaces old target");
}

static void Test_TimeWrap(void)
{
    MasterCan_t master;
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    MasterCan_Init(&master, 1U);
    MakeControlData(data, 0, 0);
    header = MakeHeader(0x101U);
    MasterCan_OnRx(&master, &header, data, 0xFFFFFFF0U);

    MasterCan_Update(&master, 0x00000010U);
    Check(master.control_seen == 1U, "time wrap control available");
    MasterCan_Update(&master, 0x000000F0U);
    Check(master.control_seen == 1U, "time wrap control persists");
    Check(master.link_valid == 1U, "time wrap link valid");
    MasterCan_Update(&master, 0x000001F0U);
    Check(master.link_valid == 0U, "time wrap link expired");
    Check(master.control_seen == 0U, "time wrap clears control");
}

int main(void)
{
    Test_WheelIds();
    Test_Decode();
    Test_InputValidation();
    Test_TimeoutAndHeartbeat();
    Test_TimeWrap();

    if (g_failures != 0U)
    {
        printf("MasterCan host tests: %u failure(s)\n",
               (unsigned)g_failures);
        return 1;
    }

    printf("MasterCan host tests: all passed\n");
    return 0;
}
