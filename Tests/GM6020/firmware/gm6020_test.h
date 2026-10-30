#ifndef GM6020_TEST_H
#define GM6020_TEST_H

#include "main.h"
#include "GM6020.h"
#include "setpoint_ramp.h"

#define GM6020_TEST_ZERO_RAW  4096U

typedef enum
{
    GM6020_TEST_DISABLED = 0,
    GM6020_TEST_ZERO_CAL,
    GM6020_TEST_POSITION_MANUAL,
    GM6020_TEST_POSITION_AUTO,
    GM6020_TEST_SPEED,
    GM6020_TEST_VOLTAGE
} GM6020TestMode_t;

typedef enum
{
    GM6020_TEST_WAVE_SINE = 0,
    GM6020_TEST_WAVE_TRIANGLE
} GM6020TestWaveform_t;

extern volatile GM6020TestMode_t g_test_mode;
extern volatile float g_test_target_deg;
extern volatile float g_test_target_rpm;
extern volatile int32_t g_test_target_voltage;
extern volatile uint8_t g_test_enable;
extern volatile uint16_t g_test_zero_raw;
extern volatile GM6020TestWaveform_t g_test_waveform;
extern volatile float g_test_wave_amplitude_deg;
extern volatile uint32_t g_test_wave_period_ms;
extern volatile uint8_t g_test_use_ramp;
extern volatile uint32_t g_ms;
extern volatile uint32_t g_can1_error_code;
extern volatile uint32_t g_can1_esr;
extern volatile uint32_t g_can1_tsr;
extern volatile uint32_t g_can1_rf0r;
extern volatile uint32_t g_can1_state;
extern volatile uint32_t g_can1_tx_free;
extern GM6020_t g_gm6020_motor;
extern SetpointRamp_t g_test_setpoint_ramp;

void GM6020Test_Init(CAN_HandleTypeDef *hcan,
                     UART_HandleTypeDef *huart);

void GM6020Test_Loop(void);

#endif /* GM6020_TEST_H */
