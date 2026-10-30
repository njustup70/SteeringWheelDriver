#include "gm6020_test.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "bsp_can.h"
#include "tim.h"
#include "usart.h"

#define TEST_LED_MASTER_PIN    GPIO_PIN_4
#define TEST_LED_VESC_PIN      GPIO_PIN_5
#define TEST_LED_ENABLE_PIN    GPIO_PIN_6
#define TEST_LED_GM6020_PIN    GPIO_PIN_7

#define TEST_SELF_TEST_MS      500U
#define TEST_TELEMETRY_MS      20U
#define TEST_AUTO_MIN_PERIOD_MS 200U
#define TEST_AUTO_MAX_AMPLITUDE_DEG 180.0f
#define TEST_AUTO_TWO_PI       6.283185307179586f

volatile GM6020TestMode_t g_test_mode = GM6020_TEST_DISABLED;
volatile float g_test_target_deg = 0.0f;
volatile float g_test_target_rpm = 50.0f;
volatile int32_t g_test_target_voltage = 0;
volatile uint8_t g_test_enable = 0U;
volatile uint16_t g_test_zero_raw = GM6020_TEST_ZERO_RAW;
volatile GM6020TestWaveform_t g_test_waveform =
    GM6020_TEST_WAVE_SINE;
volatile float g_test_wave_amplitude_deg = 30.0f;
volatile uint32_t g_test_wave_period_ms = 4000U;
volatile uint8_t g_test_use_ramp = 1U;
volatile uint32_t g_ms = 0U;
volatile uint32_t g_can1_error_code = 0U;
volatile uint32_t g_can1_esr = 0U;
volatile uint32_t g_can1_tsr = 0U;
volatile uint32_t g_can1_rf0r = 0U;
volatile uint32_t g_can1_state = 0U;
volatile uint32_t g_can1_tx_free = 0U;

GM6020_t g_gm6020_motor;
SetpointRamp_t g_test_setpoint_ramp;

static CAN_HandleTypeDef *g_can;
static UART_HandleTypeDef *g_uart;
static volatile uint8_t g_uart_busy;
static char g_uart_buffer[320];
static uint32_t g_last_telemetry_ms;

static uint8_t g_self_test_active;
static GM6020TestMode_t g_last_mode;

static volatile float g_auto_origin_deg;
static volatile float g_auto_phase;
static volatile uint8_t g_auto_use_ramp_active;
static volatile uint8_t g_auto_initialized;
static volatile uint8_t g_ramp_ready;

static void Test_LedWrite(uint16_t pin, uint8_t on)
{
    HAL_GPIO_WritePin(GPIOB,pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void Test_LedAll(uint8_t on)
{
    Test_LedWrite(TEST_LED_MASTER_PIN, on);
    Test_LedWrite(TEST_LED_VESC_PIN, on);
    Test_LedWrite(TEST_LED_ENABLE_PIN, on);
    Test_LedWrite(TEST_LED_GM6020_PIN, on);
}

static void Test_UpdateLeds(void)
{
    uint8_t heartbeat =
        (((g_ms / 500U) % 2U) == 0U) ? 1U : 0U;

    Test_LedWrite(TEST_LED_MASTER_PIN, heartbeat);
    Test_LedWrite(TEST_LED_GM6020_PIN,
                  g_gm6020_motor.is_online);
    Test_LedWrite(TEST_LED_ENABLE_PIN,
                  g_gm6020_motor.is_enabled);
    Test_LedWrite(TEST_LED_VESC_PIN,
                  ((g_test_mode == GM6020_TEST_POSITION_AUTO) ||
                   (g_test_mode == GM6020_TEST_VOLTAGE)) &&
                  g_gm6020_motor.is_enabled);
}

static float Test_ClampAbs(float value, float limit)
{
    if (limit <= 0.0f)
    {
        return 0.0f;
    }

    if (value > limit)
    {
        return limit;
    }

    if (value < -limit)
    {
        return -limit;
    }

    return value;
}

static void Test_DisableIfEnabled(void)
{
    if (g_gm6020_motor.is_enabled)
    {
        GM6020_Disable(&g_gm6020_motor);
    }
}

static void Test_UpdateTimeout(void)
{
    if (g_gm6020_motor.is_online &&
        ((uint32_t)(g_ms - g_gm6020_motor.last_rx_ms) >
         g_gm6020_motor.online_timeout_ms))
    {
        g_gm6020_motor.is_online = 0U;
        g_gm6020_motor.target_speed_rpm = 0.0f;
        g_gm6020_motor.target_voltage = 0;
        PidReset(&g_gm6020_motor.position_pid);
        PidReset(&g_gm6020_motor.speed_pid);
        (void)GM6020_SendVoltage(&g_gm6020_motor, 0);
    }
}

static void Test_SpeedControlStep(void)
{
    float voltage_cmd;

    Test_UpdateTimeout();

    if (!g_gm6020_motor.is_enabled ||
        !g_gm6020_motor.is_online)
    {
        g_gm6020_motor.target_speed_rpm = 0.0f;
        g_gm6020_motor.target_voltage = 0;
        (void)GM6020_SendVoltage(&g_gm6020_motor, 0);
        return;
    }

    g_gm6020_motor.target_speed_rpm = g_test_target_rpm;

    /* SPEED 模式绕过 GM6020_ControlStep，需要同步实际电压限幅。 */
    g_gm6020_motor.speed_pid.output_max =
        (float)g_gm6020_motor.voltage_limit_raw;

    voltage_cmd =
        PidCalPosition(&g_gm6020_motor.speed_pid,
                       g_test_target_rpm,
                       (float)g_gm6020_motor.speed_rpm);

    voltage_cmd *= (float)g_gm6020_motor.voltage_sign;
    voltage_cmd =
        Test_ClampAbs(voltage_cmd,
                      (float)g_gm6020_motor.voltage_limit_raw);

    g_gm6020_motor.target_voltage = (int16_t)voltage_cmd;
    (void)GM6020_SendVoltage(&g_gm6020_motor,
                             g_gm6020_motor.target_voltage);
}

static void Test_UpdatePositionMode(void)
{
    if (!g_test_enable || !g_gm6020_motor.is_online)
    {
        Test_DisableIfEnabled();
        return;
    }

    if (!g_gm6020_motor.is_enabled)
    {
        GM6020_Enable(&g_gm6020_motor);
        SetpointRamp_Reset(&g_test_setpoint_ramp,
                           g_gm6020_motor.angle_deg);
        g_ramp_ready = 1U;
    }
}

static void Test_UpdateManualMode(void)
{
    Test_UpdatePositionMode();
}

static void Test_UpdateAutoMode(void)
{
    uint8_t use_ramp;

    Test_UpdatePositionMode();

    if (!g_gm6020_motor.is_enabled)
    {
        g_auto_initialized = 0U;
        return;
    }

    use_ramp = (g_test_use_ramp != 0U) ? 1U : 0U;
    if (use_ramp != g_auto_use_ramp_active)
    {
        g_auto_use_ramp_active = use_ramp;
        g_auto_initialized = 0U;
        g_ramp_ready = 0U;
    }

    if (!g_auto_initialized)
    {
        g_auto_origin_deg = g_gm6020_motor.angle_deg;
        g_auto_phase = 0.0f;
        g_test_target_deg = g_auto_origin_deg;
        GM6020_SetTargetAngle(&g_gm6020_motor,
                              g_auto_origin_deg);

        if (g_gm6020_motor.is_online)
        {
            SetpointRamp_Reset(&g_test_setpoint_ramp,
                               g_auto_origin_deg);
            g_ramp_ready = 1U;
        }

        g_auto_initialized = 1U;
    }
}

static void Test_UpdateAutoTarget(void)
{
    float amplitude_deg;
    float phase;
    float wave;
    float target_deg;
    uint32_t period_ms;
    uint8_t use_ramp;

    if (!g_gm6020_motor.is_enabled || !g_auto_initialized)
    {
        return;
    }

    use_ramp = (g_test_use_ramp != 0U) ? 1U : 0U;
    if (use_ramp != g_auto_use_ramp_active)
    {
        /* 下一轮主循环会以当前位置重新初始化波形和斜坡。 */
        g_auto_use_ramp_active = use_ramp;
        g_auto_initialized = 0U;
        g_ramp_ready = 0U;
        return;
    }

    period_ms = g_test_wave_period_ms;
    if (period_ms < TEST_AUTO_MIN_PERIOD_MS)
    {
        period_ms = TEST_AUTO_MIN_PERIOD_MS;
    }

    g_auto_phase += 1.0f / (float)period_ms;
    if (g_auto_phase >= 1.0f)
    {
        g_auto_phase -= 1.0f;
    }

    phase = g_auto_phase;
    if (g_test_waveform == GM6020_TEST_WAVE_TRIANGLE)
    {
        if (phase < 0.25f)
        {
            wave = 4.0f * phase;
        }
        else if (phase < 0.75f)
        {
            wave = 2.0f - (4.0f * phase);
        }
        else
        {
            wave = (4.0f * phase) - 4.0f;
        }
    }
    else
    {
        wave = sinf(TEST_AUTO_TWO_PI * phase);
    }

    amplitude_deg = fabsf(g_test_wave_amplitude_deg);
    if (amplitude_deg > TEST_AUTO_MAX_AMPLITUDE_DEG)
    {
        amplitude_deg = TEST_AUTO_MAX_AMPLITUDE_DEG;
    }

    target_deg = g_auto_origin_deg + (amplitude_deg * wave);
    g_test_target_deg = target_deg;

    if (g_auto_use_ramp_active)
    {
        SetpointRamp_SetDesired(&g_test_setpoint_ramp,
                                target_deg);
        SetpointRamp_Update(&g_test_setpoint_ramp, 0.001f);
        GM6020_SetTargetAngle(
            &g_gm6020_motor,
            SetpointRamp_GetOutput(&g_test_setpoint_ramp));
    }
    else
    {
        GM6020_SetTargetAngle(&g_gm6020_motor, target_deg);
    }
}

static void Test_UpdateSpeedMode(void)
{
    if (!g_test_enable || !g_gm6020_motor.is_online)
    {
        Test_DisableIfEnabled();
        return;
    }

    if (!g_gm6020_motor.is_enabled)
    {
        GM6020_Enable(&g_gm6020_motor);
    }
}

static void Test_UpdateVoltageMode(void)
{
    if (!g_test_enable || !g_gm6020_motor.is_online)
    {
        Test_DisableIfEnabled();
        return;
    }

    if (!g_gm6020_motor.is_enabled)
    {
        GM6020_Enable(&g_gm6020_motor);
    }
}

static void Test_VoltageControlStep(void)
{
    float voltage_cmd;

    Test_UpdateTimeout();

    if (!g_gm6020_motor.is_enabled ||
        !g_gm6020_motor.is_online)
    {
        g_gm6020_motor.target_speed_rpm = 0.0f;
        g_gm6020_motor.target_voltage = 0;
        (void)GM6020_SendVoltage(&g_gm6020_motor, 0);
        return;
    }

    voltage_cmd =
        Test_ClampAbs((float)g_test_target_voltage,
                      (float)g_gm6020_motor.voltage_limit_raw);

    g_gm6020_motor.target_speed_rpm = 0.0f;
    g_gm6020_motor.target_voltage = (int16_t)voltage_cmd;
    (void)GM6020_SendVoltage(&g_gm6020_motor,
                             g_gm6020_motor.target_voltage);
}

static void Test_UpdatePositionRamp(void)
{
    if (!g_gm6020_motor.is_online)
    {
        g_ramp_ready = 0U;
        return;
    }

    if (!g_ramp_ready)
    {
        SetpointRamp_Reset(&g_test_setpoint_ramp,
                           g_gm6020_motor.angle_deg);
        g_ramp_ready = 1U;
    }

    SetpointRamp_SetDesired(&g_test_setpoint_ramp,
                            g_test_target_deg);
    SetpointRamp_Update(&g_test_setpoint_ramp, 0.001f);
    GM6020_SetTargetAngle(&g_gm6020_motor,
                          SetpointRamp_GetOutput(
                              &g_test_setpoint_ramp));
}

static void Test_SendTelemetry(void)
{
    int target_deg_x100;
    int actual_deg_x100;
    int target_speed_rpm;
    int actual_speed_rpm;
    int target_voltage;
    int actual_current;
    int last_tx_status;
    int last_tx_voltage;
    int last_tx_data0;
    int last_tx_data1;
    int raw_wave_deg_x100;
    int length;

    if ((g_uart == NULL) || g_uart_busy)
    {
        return;
    }

    target_deg_x100 =
        (int)(g_gm6020_motor.target_angle_deg * 100.0f);
    actual_deg_x100 =
        (int)(g_gm6020_motor.angle_deg * 100.0f);
    target_speed_rpm =
        (int)g_gm6020_motor.target_speed_rpm;
    actual_speed_rpm = (int)g_gm6020_motor.speed_rpm;
    target_voltage = (int)g_gm6020_motor.target_voltage;
    actual_current = (int)g_gm6020_motor.current_raw;
    last_tx_status = (int)g_gm6020_motor.last_tx_status;
    last_tx_voltage = (int)g_gm6020_motor.last_tx_voltage;
    last_tx_data0 = (int)g_gm6020_motor.last_tx_data[0];
    last_tx_data1 = (int)g_gm6020_motor.last_tx_data[1];
    raw_wave_deg_x100 = (int)(g_test_target_deg * 100.0f);

    length = snprintf(
        g_uart_buffer,
        sizeof(g_uart_buffer),
        "%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,"
        "%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,"
        "%u,%u,%d,%d,%d,%d\r\n",
        target_deg_x100,
        actual_deg_x100,
        target_speed_rpm,
        actual_speed_rpm,
        target_voltage,
        actual_current,
        (unsigned)g_gm6020_motor.is_online,
        (unsigned)g_gm6020_motor.is_enabled,
        (unsigned)g_test_mode,
        (unsigned)g_gm6020_motor.raw_angle,
        last_tx_status,
        (unsigned)g_gm6020_motor.tx_ok_count,
        (unsigned)g_gm6020_motor.tx_busy_count,
        (unsigned)g_gm6020_motor.tx_error_count,
        (unsigned)g_can1_error_code,
        (unsigned)g_can1_esr,
        (unsigned)g_can1_tsr,
        (unsigned)g_can1_rf0r,
        (unsigned)g_can1_state,
        (unsigned)g_can1_tx_free,
        (unsigned)g_gm6020_motor.temperature,
        (int)g_gm6020_motor.last_tx_std_id,
        last_tx_voltage,
        last_tx_data0,
        last_tx_data1,
        raw_wave_deg_x100);

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

void GM6020Test_Init(CAN_HandleTypeDef *hcan,
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
    g_test_mode = GM6020_TEST_DISABLED;
    g_test_target_deg = 0.0f;
    g_test_target_rpm = 50.0f;
    g_test_target_voltage = 0;
    g_test_enable = 0U;
    g_test_zero_raw = GM6020_TEST_ZERO_RAW;
    g_test_waveform = GM6020_TEST_WAVE_SINE;
    g_test_wave_amplitude_deg = 30.0f;
    g_test_wave_period_ms = 4000U;
    g_test_use_ramp = 1U;
    g_ms = 0U;

    g_last_mode = GM6020_TEST_DISABLED;
    g_self_test_active = 1U;
    g_auto_use_ramp_active = 1U;
    g_auto_initialized = 0U;
    g_ramp_ready = 0U;

    SetpointRamp_Init(&g_test_setpoint_ramp,
                      0.0f,
                      SETPOINT_RAMP_DEFAULT_MAX_SPEED_DEG_S,
                      SETPOINT_RAMP_DEFAULT_MAX_ACCEL_DEG_S2);

    GM6020_Init(&g_gm6020_motor,
                hcan,
                1U,
                g_test_zero_raw,
                1,
                1,
                1.0f);

    g_gm6020_motor.voltage_limit_raw =
        GM6020_DEFAULT_VOLTAGE_LIMIT;
    g_gm6020_motor.max_speed_rpm =
        GM6020_DEFAULT_MAX_SPEED_RPM;
    g_gm6020_motor.online_timeout_ms =
        GM6020_DEFAULT_TIMEOUT_MS;

    GM6020_Disable(&g_gm6020_motor);

    Test_LedAll(1U);

    HAL_NVIC_SetPriority(USART1_IRQn, 3U, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void GM6020Test_Loop(void)
{
    uint16_t zero_raw;

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
        if (g_ms < TEST_SELF_TEST_MS)
        {
            Test_LedAll(1U);
            return;
        }

        g_self_test_active = 0U;
        Test_LedAll(0U);
    }

    zero_raw = (uint16_t)(g_test_zero_raw & 0x1FFFU);
    g_test_zero_raw = zero_raw;

    if (g_gm6020_motor.zero_raw != zero_raw)
    {
        g_gm6020_motor.zero_raw = zero_raw;

        if (g_test_mode != GM6020_TEST_ZERO_CAL)
        {
            GM6020_Disable(&g_gm6020_motor);
            g_test_enable = 0U;
        }
    }

    if (g_test_mode != g_last_mode)
    {
        g_last_mode = g_test_mode;
        g_auto_initialized = 0U;
        GM6020_Disable(&g_gm6020_motor);

        if ((g_test_mode == GM6020_TEST_POSITION_MANUAL) ||
            (g_test_mode == GM6020_TEST_POSITION_AUTO))
        {
            if (g_gm6020_motor.is_online)
            {
                SetpointRamp_Reset(&g_test_setpoint_ramp,
                                   g_gm6020_motor.angle_deg);
                g_ramp_ready = 1U;
            }
            else
            {
                g_ramp_ready = 0U;
            }

            if (g_test_mode == GM6020_TEST_POSITION_AUTO)
            {
                g_test_target_deg = g_gm6020_motor.angle_deg;
            }
        }

        if ((g_test_mode == GM6020_TEST_DISABLED) ||
            (g_test_mode == GM6020_TEST_ZERO_CAL))
        {
            g_test_enable = 0U;
        }
    }

    switch (g_test_mode)
    {
        case GM6020_TEST_DISABLED:
        case GM6020_TEST_ZERO_CAL:
            Test_DisableIfEnabled();
            break;

        case GM6020_TEST_POSITION_MANUAL:
            Test_UpdateManualMode();
            break;

        case GM6020_TEST_POSITION_AUTO:
            Test_UpdateAutoMode();
            break;

        case GM6020_TEST_SPEED:
            Test_UpdateSpeedMode();
            break;

        case GM6020_TEST_VOLTAGE:
            Test_UpdateVoltageMode();
            break;

        default:
            g_test_mode = GM6020_TEST_DISABLED;
            g_test_enable = 0U;
            Test_DisableIfEnabled();
            break;
    }

    Test_UpdateLeds();

    if ((uint32_t)(g_ms - g_last_telemetry_ms) >=
        TEST_TELEMETRY_MS)
    {
        g_last_telemetry_ms = g_ms;
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

        GM6020_OnRx(&g_gm6020_motor,
                    &header,
                    data,
                    g_ms);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if ((htim != NULL) && (htim->Instance == TIM6))
    {
        g_ms++;

        if (g_test_mode == GM6020_TEST_SPEED)
        {
            Test_SpeedControlStep();
        }
        else if (g_test_mode == GM6020_TEST_VOLTAGE)
        {
            Test_VoltageControlStep();
        }
        else
        {
            if (g_test_mode == GM6020_TEST_POSITION_AUTO)
            {
                Test_UpdateAutoTarget();
            }
            else if (g_test_mode == GM6020_TEST_POSITION_MANUAL)
            {
                Test_UpdatePositionRamp();
            }

            GM6020_ControlStep(&g_gm6020_motor, g_ms);
        }
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
