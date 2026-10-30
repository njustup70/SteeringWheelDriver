#ifndef MASTERCAN_TEST_H
#define MASTERCAN_TEST_H

#include "main.h"
#include "MasterCan.h"

#define MASTER_CAN_TEST_WHEEL_ID  1U

extern MasterCan_t g_master_can;
extern volatile uint32_t g_master_can_test_ms;
extern volatile uint32_t g_mastercan_control_rx_count;
extern volatile uint32_t g_mastercan_heartbeat_rx_count;
extern volatile uint32_t g_mastercan_invalid_rx_count;
extern volatile uint32_t g_can2_error_code;
extern volatile uint32_t g_can2_esr;
extern volatile uint32_t g_can2_tsr;
extern volatile uint32_t g_can2_rf0r;
extern volatile uint32_t g_can2_state;
extern volatile uint32_t g_can2_rx_fill;

void MasterCanTest_Init(CAN_HandleTypeDef *hcan,
                        UART_HandleTypeDef *huart);

void MasterCanTest_Loop(void);

#endif /* MASTERCAN_TEST_H */
