#ifndef VESC_H
#define VESC_H

#include <stdint.h>

#include "main.h"

/**
 * @file VESC.h
 * @brief VESC 扩展帧 CAN 驱动。
 *
 * 控制使用 CAN_PACKET_SET_RPM，状态接收 CAN_PACKET_STATUS。
 * ERPM 是电气转速，不是机械 RPM。
 */

#define VESC_PACKET_SET_RPM       3U
#define VESC_PACKET_STATUS        9U
#define VESC_DEFAULT_TIMEOUT_MS   100U

typedef struct
{
    /* 初始化配置 */
    CAN_HandleTypeDef *hcan;
    uint8_t controller_id;

    /* 控制和反馈状态 */
    int32_t target_erpm;
    int32_t actual_erpm;
    float actual_current_a;
    float duty;
    uint32_t last_rx_ms;
    uint32_t online_timeout_ms;
    uint8_t is_online;
    uint8_t is_enabled;

    /* 最近一次发送诊断 */
    HAL_StatusTypeDef last_tx_status;
    uint32_t tx_ok_count;
    uint32_t tx_busy_count;
    uint32_t tx_error_count;
    uint32_t last_tx_id;
    int32_t last_tx_erpm;
    uint8_t last_tx_data[8];
} VESC_t;

/** @brief 初始化 VESC 上下文和默认通讯超时。 */
void VESC_Init(VESC_t *vesc, CAN_HandleTypeDef *hcan, uint8_t controller_id);

/** @brief 解析扩展状态帧 (9<<8)|controller_id。 */
void VESC_OnRx(VESC_t *vesc, const CAN_RxHeaderTypeDef *header, const uint8_t data[8], uint32_t now_ms);

/** @brief 设置下一次周期控制要发送的 ERPM。 */
void VESC_SetTargetErpm(VESC_t *vesc, int32_t erpm);

/** @brief 使能 VESC，初始目标为 0。 */
void VESC_Enable(VESC_t *vesc);

/** @brief 失能并立即发送 0 ERPM。 */
void VESC_Disable(VESC_t *vesc);

/**
 * @brief 执行一次周期控制。
 * @note 离线或失能时仍发送 0 ERPM；在线使能时发送 target_erpm。
 */
void VESC_ControlStep(VESC_t *vesc, uint32_t now_ms);

/** @brief 立即发送一帧大端 int32 ERPM 指令。 */
HAL_StatusTypeDef VESC_SendErpm(VESC_t *vesc, int32_t erpm);

#endif /* VESC_H */
