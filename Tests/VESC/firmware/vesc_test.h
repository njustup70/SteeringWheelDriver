#ifndef VESC_TEST_H
#define VESC_TEST_H

#include "main.h"
#include "VESC.h"

#define VESC_TEST_CONTROLLER_ID                  67U
#define VESC_TEST_DEFAULT_ERPM_LIMIT             1000

typedef enum
{
    VESC_TEST_DISABLED = 0,
    VESC_TEST_STEP
} VESCTestMode_t;

extern volatile VESCTestMode_t g_vesc_test_mode;
extern volatile uint8_t g_vesc_test_enable;
extern volatile int32_t g_vesc_test_target_erpm;
extern volatile int32_t g_vesc_test_erpm_limit;
extern volatile uint32_t g_vesc_test_ms;
extern volatile uint32_t g_can1_error_code;
extern volatile uint32_t g_can1_esr;
extern volatile uint32_t g_can1_tsr;
extern volatile uint32_t g_can1_rf0r;
extern volatile uint32_t g_can1_state;
extern volatile uint32_t g_can1_tx_free;

extern VESC_t g_vesc_motor;

void VESCTest_Init(CAN_HandleTypeDef *hcan,
                   UART_HandleTypeDef *huart);

void VESCTest_Loop(void);

#endif /* VESC_TEST_H */
