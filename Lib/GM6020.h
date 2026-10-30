#ifndef GM6020_H
#define GM6020_H

#include <stddef.h>

#include "main.h"
#include "pid.h"

/**
 * @file GM6020.h
 * @brief GM6020 电压模式驱动、单圈角度解绕和串级 PID 控制。
 *
 * 控制链：
 *   目标舵轮角 -> 位置 PID -> 目标 RPM -> 速度 PID -> 电压 raw
 *
 * 角度反馈只支持单圈 0..8191 编码器值。当前机械方案为直驱；
 * gear_ratio 字段保留给后续扩展，但减速结构不能只依赖单圈电机角。
 */

#define GM6020_RAW_PER_REV              8192.0f
#define GM6020_DEG_PER_REV              360.0f

#define GM6020_DEFAULT_CONTROL_DT       0.001f
#define GM6020_DEFAULT_MAX_SPEED_RPM    300.0f
#define GM6020_DEFAULT_VOLTAGE_LIMIT    20000
#define GM6020_MAX_VOLTAGE_RAW          25000
#define GM6020_DEFAULT_TIMEOUT_MS       20U

#define GM6020_DEFAULT_POS_KP           20.0f
#define GM6020_DEFAULT_POS_KI           2.0f
#define GM6020_DEFAULT_POS_KD           0.1f
#define GM6020_DEFAULT_POS_INTEGRAL_MAX 50.0f
#define GM6020_DEFAULT_POS_SEP_DEG      10.0f

#define GM6020_DEFAULT_SPD_KP           70.0f
#define GM6020_DEFAULT_SPD_KI           50.0f
#define GM6020_DEFAULT_SPD_KD           0.0f
#define GM6020_DEFAULT_SPD_INTEGRAL_MAX 8000.0f
#define GM6020_DEFAULT_SPD_SEP_RPM      100.0f

typedef struct
{
    /* 初始化配置 */
    CAN_HandleTypeDef *hcan;
    uint8_t id;                 /* GM6020 ID: 1..7 */
    uint16_t zero_raw;          /* 机械零位对应的原始角: 0..8191 */
    int8_t angle_sign;          /* 编码器角度正方向: +1 / -1 */
    int8_t voltage_sign;        /* 电压正方向: +1 / -1 */
    float gear_ratio;           /* 电机转数/舵轮转数，当前直驱应为 1.0 */

    /* 反馈状态 */
    uint16_t raw_angle;         /* GM6020 原始机械角: 0..8191 */
    float angle_deg;            /* 减零位并定方向后的舵轮角: 0..360 */
    int16_t speed_rpm;          /* 已换算到舵轮正方向的反馈转速 */
    int16_t current_raw;        /* GM6020 反馈的转矩电流 raw */
    uint8_t temperature;        /* GM6020 反馈温度 */
    uint8_t is_online;          /* 是否在超时时间内收到反馈 */
    uint8_t is_enabled;         /* 是否允许位置环输出 */
    uint32_t last_rx_ms;        /* 上次收到反馈的时间 */

    float target_angle_deg;     /* 目标舵轮角: 0..360 */
    float target_speed_rpm;     /* 位置环输出的目标转速 */
    int16_t target_voltage;     /* 最终发给 GM6020 的电压 raw */

    /* 最近一次电压帧发送诊断 */
    HAL_StatusTypeDef last_tx_status;
    uint32_t tx_ok_count;
    uint32_t tx_busy_count;
    uint32_t tx_error_count;
    uint32_t last_tx_std_id;
    int16_t last_tx_voltage;
    uint8_t last_tx_data[8];

    /* 运行时控制限幅 */
    float max_speed_rpm;        /* 位置环输出限幅 */
    int16_t voltage_limit_raw;  /* 最终电压限幅 */
    uint32_t online_timeout_ms; /* 反馈超时时间 */

    /* 两级串级控制器 */
    Pids position_pid;
    Pids speed_pid;
} GM6020_t;

/**
 * @brief 初始化 GM6020 上下文、默认限幅和两级 PID。
 * @param id GM6020 ID，范围 1..7。
 * @param zero_raw 机械零位对应的原始角，自动限制到 13 位范围。
 * @param angle_sign 编码器角度正方向，传入负值会规范化为 -1。
 * @param voltage_sign 电压正方向，传入负值会规范化为 -1。
 * @param gear_ratio 电机转数/舵轮转数；非正值回退为 1.0。
 */
void GM6020_Init(GM6020_t *motor,
                 CAN_HandleTypeDef *hcan,
                 uint8_t id,
                 uint16_t zero_raw,
                 int8_t angle_sign,
                 int8_t voltage_sign,
                 float gear_ratio);

/**
 * @brief 解析一帧 GM6020 反馈并更新时间戳和在线状态。
 * @note 期望标准数据帧 0x204+id，DLC 8。首次重新上线时重置目标与 PID。
 */
void GM6020_OnRx(GM6020_t *motor,
                 const CAN_RxHeaderTypeDef *header,
                 const uint8_t data[8],
                 uint32_t now_ms);

/** @brief 设置舵轮目标角，内部归一化到 0..360。 */
void GM6020_SetTargetAngle(GM6020_t *motor, float angle_deg);

/** @brief 使能控制，并把目标角重置为当前反馈角以避免使能跳变。 */
void GM6020_Enable(GM6020_t *motor);

/** @brief 失能、清 PID，并立即发送一次零电压。 */
void GM6020_Disable(GM6020_t *motor);

/**
 * @brief 执行一次 1 kHz 串级控制。
 * @note 反馈超时、失能或离线时仍会调用发送流程，确保总线上持续输出零电压。
 */
void GM6020_ControlStep(GM6020_t *motor, uint32_t now_ms);

/** @brief 将电压命令打包并发送到对应 GM6020 电压控制槽位。 */
HAL_StatusTypeDef GM6020_SendVoltage(GM6020_t *motor, int16_t voltage_raw);

/** @brief 将角度归一化到 [-180, 180)，用于最短角度误差。 */
float GM6020_Wrap180(float angle_deg);

#endif /* GM6020_H */
