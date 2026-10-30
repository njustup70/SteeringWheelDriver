#include "GM6020.h"

#include <math.h>
#include <string.h>

/** @brief 把角度归一化到 [0, 360)。 */
static float GM6020_Wrap360(float angle_deg)
{
    angle_deg = fmodf(angle_deg, GM6020_DEG_PER_REV);
    if (angle_deg < 0.0f)
    {
        angle_deg += GM6020_DEG_PER_REV;
    }
    return angle_deg;
}

/** @brief 将值限制在 [-limit, +limit]；非正 limit 返回 0。 */
static float GM6020_ClampAbs(float value, float limit)
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

/**
 * @brief 把单圈原始编码器角换算为舵轮本地坐标角。
 *
 * 先减机械零位，再乘编码器方向和角度分辨率。当前直驱方案 gear_ratio=1；
 * 若以后使用减速器，单圈电机角不能唯一表示舵轮多圈绝对位置。
 */
static float GM6020_RawToAngle(const GM6020_t *motor, uint16_t raw_angle)
{
    int32_t delta;
    float motor_deg;
    delta = (int32_t)raw_angle - (int32_t)motor->zero_raw;
    motor_deg =(float)delta * (GM6020_DEG_PER_REV / GM6020_RAW_PER_REV) * (float)motor->angle_sign;

    if (motor->gear_ratio > 0.0f && motor->gear_ratio != 1.0f)
    {
        /*
         * 当前机械方案是直驱。保留 gear_ratio 供后续扩展；
         * 减速传动需要多圈解绕或舵轮绝对编码器，不能只靠单圈电机角。
         */
        motor_deg /= motor->gear_ratio;
    }

    return GM6020_Wrap360(motor_deg);
}

/**
 * @brief 把 PID 输出限幅同步到当前运行时最大速度和电压限制。
 * @note 条件积分抗饱和依赖 output_max，因此运行中修改限制后必须每周期同步。
 */
static void GM6020_UpdateControlLimits(GM6020_t *motor)
{
    motor->position_pid.output_max = fabsf(motor->max_speed_rpm);
    motor->speed_pid.output_max =
        fabsf((float)motor->voltage_limit_raw);
}

/** @brief 初始化反馈状态、发送诊断和两级控制器参数。 */
void GM6020_Init(GM6020_t *motor, CAN_HandleTypeDef *hcan, uint8_t id, uint16_t zero_raw, int8_t angle_sign, int8_t voltage_sign, float gear_ratio)
{
    if ((motor == NULL) || (hcan == NULL) || (id < 1U) || (id > 7U))
    {
        Error_Handler();
    }

    motor->hcan = hcan;
    motor->id = id;
    motor->zero_raw = (uint16_t)(zero_raw & 0x1FFFU);
    motor->angle_sign = (angle_sign < 0) ? -1 : 1;
    motor->voltage_sign = (voltage_sign < 0) ? -1 : 1;
    /* 非正传动比会使角度换算失真，直接回退为直驱。 */
    motor->gear_ratio = (gear_ratio > 0.0f) ? gear_ratio : 1.0f;

    motor->raw_angle = 0U;
    motor->angle_deg = 0.0f;
    motor->speed_rpm = 0;
    motor->current_raw = 0;
    motor->temperature = 0U;
    motor->is_online = 0U;
    motor->is_enabled = 0U;
    motor->last_rx_ms = 0U;

    motor->target_angle_deg = 0.0f;
    motor->target_speed_rpm = 0.0f;
    motor->target_voltage = 0;

    motor->last_tx_status = HAL_ERROR;
    motor->tx_ok_count = 0U;
    motor->tx_busy_count = 0U;
    motor->tx_error_count = 0U;
    motor->last_tx_std_id = 0U;
    motor->last_tx_voltage = 0;
    memset(motor->last_tx_data, 0, sizeof(motor->last_tx_data));

    motor->max_speed_rpm = GM6020_DEFAULT_MAX_SPEED_RPM;
    motor->voltage_limit_raw = GM6020_DEFAULT_VOLTAGE_LIMIT;
    motor->online_timeout_ms = GM6020_DEFAULT_TIMEOUT_MS;

    PidInit(&motor->position_pid,
            GM6020_DEFAULT_POS_KP,
            GM6020_DEFAULT_POS_KI,
            GM6020_DEFAULT_POS_KD,
            GM6020_DEFAULT_CONTROL_DT,
            motor->max_speed_rpm,
            GM6020_DEFAULT_POS_INTEGRAL_MAX,
            1U, /* 位置环保留积分分离，避免大角度阶段积分堆积。 */
            GM6020_DEFAULT_POS_SEP_DEG);

    PidInit(&motor->speed_pid,
            GM6020_DEFAULT_SPD_KP,
            GM6020_DEFAULT_SPD_KI,
            GM6020_DEFAULT_SPD_KD,
            GM6020_DEFAULT_CONTROL_DT,
            (float)motor->voltage_limit_raw,
            GM6020_DEFAULT_SPD_INTEGRAL_MAX,
            0U, /* 速度积分持续工作，用于克服低速静摩擦。 */
            GM6020_DEFAULT_SPD_SEP_RPM);
}

/**
 * @brief 解码 GM6020 反馈帧 0x204+id。
 *
 * 机械角、转速、电流均为大端 16 位，温度位于 Data[6]。
 * 角度先减去软件零位并应用编码器方向，再归一化到单圈 0..360。
 */
void GM6020_OnRx(GM6020_t *motor, const CAN_RxHeaderTypeDef *header, const uint8_t data[8], uint32_t now_ms)
{
    uint16_t raw_angle;
    int16_t raw_speed;
    uint8_t was_online;

    if ((motor == NULL) || (header == NULL) || (data == NULL))
    {
        return;
    }

    if ((header->IDE != CAN_ID_STD) || (header->RTR != CAN_RTR_DATA) || (header->DLC != 8U) || (header->StdId != (uint32_t)(0x204U + motor->id)))
    {
        return;
    }

    raw_angle = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    raw_speed = (int16_t)(((uint16_t)data[2] << 8) | data[3]);

    was_online = motor->is_online;

    motor->raw_angle = raw_angle;
    motor->angle_deg = GM6020_RawToAngle(motor, raw_angle);
    motor->speed_rpm = (int16_t)((int32_t)motor->angle_sign * (int32_t)raw_speed);
    motor->current_raw = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    motor->temperature = data[6];
    motor->last_rx_ms = now_ms;
    
    /* 重新上线时把目标拉回当前角，避免继续使用失联前的旧目标。 */
    if (!was_online)
    {
        motor->target_angle_deg = motor->angle_deg;
        motor->target_speed_rpm = 0.0f;
        motor->target_voltage = 0;
        PidReset(&motor->position_pid);
        PidReset(&motor->speed_pid);
    }

    motor->is_online = 1U;
}

/** @brief 设置目标角并归一化到单圈 [0, 360)。 */
void GM6020_SetTargetAngle(GM6020_t *motor, float angle_deg)
{
    if (motor == NULL)
    {
        return;
    }

    motor->target_angle_deg = GM6020_Wrap360(angle_deg);
}

/**
 * @brief 使能 GM6020 控制。
 * @note 在线时以当前实际角作为新目标；清空 PID 可避免上次运行积分残留。
 */
void GM6020_Enable(GM6020_t *motor)
{
    if (motor == NULL)
    {
        return;
    }

    if (motor->is_online)
    {
        motor->target_angle_deg = motor->angle_deg;
    }

    motor->target_speed_rpm = 0.0f;
    motor->target_voltage = 0;
    PidReset(&motor->position_pid);
    PidReset(&motor->speed_pid);
    motor->is_enabled = 1U;
}

/** @brief 失能 GM6020，并同步清除目标速度、目标和 PID 状态。 */
void GM6020_Disable(GM6020_t *motor)
{
    if (motor == NULL)
    {
        return;
    }

    motor->is_enabled = 0U;
    motor->target_speed_rpm = 0.0f;
    motor->target_voltage = 0;

    if (motor->is_online)
    {
        motor->target_angle_deg = motor->angle_deg;
    }

    PidReset(&motor->position_pid);
    PidReset(&motor->speed_pid);
    (void)GM6020_SendVoltage(motor, 0);
}

/**
 * @brief 1 kHz 位置-速度串级控制。
 *
 * 位置误差经过 wrap180 后送入位置 PID，其输出作为目标 RPM；
 * 速度 PID 将目标 RPM 与反馈 RPM 的误差转换为电压 raw。
 */
void GM6020_ControlStep(GM6020_t *motor, uint32_t now_ms)
{
    float error_deg;
    float target_rpm;
    float voltage_cmd;

    if (motor == NULL)
    {
        return;
    }

    if (motor->is_online &&
        ((uint32_t)(now_ms - motor->last_rx_ms) >
         motor->online_timeout_ms))
    {
        /* 反馈超时：清状态、清积分并立即输出零电压。 */
        motor->is_online = 0U;
        motor->target_speed_rpm = 0.0f;
        motor->target_voltage = 0;
        PidReset(&motor->position_pid);
        PidReset(&motor->speed_pid);
        (void)GM6020_SendVoltage(motor, 0);
        return;
    }

    if (!motor->is_enabled || !motor->is_online)
    {
        motor->target_speed_rpm = 0.0f;
        motor->target_voltage = 0;
        (void)GM6020_SendVoltage(motor, 0);
        return;
    }

    /*
     * output_max 参与条件积分抗饱和，必须与当前实际限幅一致。
     * 这里每周期同步，允许测试或应用在运行时修改限幅。
     */
    GM6020_UpdateControlLimits(motor);

    /* 使用最短角误差，使 359° 与 1° 之间不会被误判为整圈旋转。 */
    error_deg =
        GM6020_Wrap180(motor->target_angle_deg - motor->angle_deg);

    /* 位置环输出目标转速，速度环再输出最终电压。 */
    target_rpm =
        PidCalPositionError(&motor->position_pid, error_deg);
    target_rpm =
        GM6020_ClampAbs(target_rpm, motor->max_speed_rpm);
    motor->target_speed_rpm = target_rpm;

    voltage_cmd =
        PidCalPosition(&motor->speed_pid,
                       target_rpm,
                       (float)motor->speed_rpm);
    voltage_cmd *= (float)motor->voltage_sign;
    voltage_cmd =
        GM6020_ClampAbs(voltage_cmd,
                        (float)motor->voltage_limit_raw);

    motor->target_voltage = (int16_t)voltage_cmd;
    (void)GM6020_SendVoltage(motor, motor->target_voltage);
}

/**
 * @brief 发送 GM6020 电压控制帧。
 *
 * ID 1..4 使用 0x1FF，ID 5..7 使用 0x2FF。每个 ID 占两个字节，
 * 大端有符号电压 raw 被放到对应槽位，其余槽位保持 0。
 */
HAL_StatusTypeDef GM6020_SendVoltage(GM6020_t *motor, int16_t voltage_raw)
{
    uint8_t data[8] = {0};
    uint32_t frame_id;
    uint32_t tx_mailbox;
    uint8_t slot;
    int32_t limited_voltage;
    CAN_TxHeaderTypeDef header = {0};

    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    if ((motor->hcan == NULL) ||
        (motor->id < 1U) ||
        (motor->id > 7U))
    {
        motor->last_tx_status = HAL_ERROR;
        motor->tx_error_count++;
        return HAL_ERROR;
    }

    limited_voltage = voltage_raw;
    if (limited_voltage > GM6020_MAX_VOLTAGE_RAW)
    {
        limited_voltage = GM6020_MAX_VOLTAGE_RAW;
    }
    else if (limited_voltage < -GM6020_MAX_VOLTAGE_RAW)
    {
        limited_voltage = -GM6020_MAX_VOLTAGE_RAW;
    }

    if (motor->id <= 4U)
    {
        frame_id = 0x1FFU;
        slot = (uint8_t)(motor->id - 1U);
    }
    else
    {
        frame_id = 0x2FFU;
        slot = (uint8_t)(motor->id - 5U);
    }

    data[slot * 2U] = (uint8_t)((uint16_t)limited_voltage >> 8);
    data[slot * 2U + 1U] = (uint8_t)((uint16_t)limited_voltage & 0xFFU);

    header.StdId = frame_id;
    header.ExtId = 0U;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    header.TransmitGlobalTime = DISABLE;

    /* 保存最近一次发送内容，便于 Watch 和串口诊断。 */
    motor->last_tx_std_id = frame_id;
    motor->last_tx_voltage = (int16_t)limited_voltage;

    motor->last_tx_status =
        HAL_CAN_AddTxMessage(motor->hcan, &header, data, &tx_mailbox);

    memcpy(motor->last_tx_data, data, sizeof(motor->last_tx_data));

    if (motor->last_tx_status == HAL_OK)
    {
        motor->tx_ok_count++;
    }
    else if (motor->last_tx_status == HAL_BUSY)
    {
        motor->tx_busy_count++;
    }
    else
    {
        motor->tx_error_count++;
    }

    return motor->last_tx_status;
}

/** @brief 把角度归一化到 [-180, 180)，用于选择最短角误差。 */
float GM6020_Wrap180(float angle_deg)
{
    angle_deg = GM6020_Wrap360(angle_deg);
    if (angle_deg >= 180.0f)
    {
        angle_deg -= 360.0f;
    }
    return angle_deg;
}
