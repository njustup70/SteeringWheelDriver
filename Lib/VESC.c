#include "VESC.h"

#include <string.h>

/** @brief 初始化状态、默认 100 ms 在线超时和发送诊断。 */
void VESC_Init(VESC_t *vesc, CAN_HandleTypeDef *hcan, uint8_t controller_id)
{
    if ((vesc == NULL) || (hcan == NULL))
    {
        Error_Handler();
    }

    vesc->hcan = hcan;
    vesc->controller_id = controller_id;

    vesc->target_erpm = 0;
    vesc->actual_erpm = 0;
    vesc->actual_current_a = 0.0f;
    vesc->duty = 0.0f;
    vesc->last_rx_ms = 0U;
    vesc->online_timeout_ms = VESC_DEFAULT_TIMEOUT_MS;
    vesc->is_online = 0U;
    vesc->is_enabled = 0U;

    vesc->last_tx_status = HAL_ERROR;
    vesc->tx_ok_count = 0U;
    vesc->tx_busy_count = 0U;
    vesc->tx_error_count = 0U;
    vesc->last_tx_id = 0U;
    vesc->last_tx_erpm = 0;
    memset(vesc->last_tx_data, 0, sizeof(vesc->last_tx_data));
}

/**
 * @brief 解析 VESC STATUS 帧。
 *
 * STATUS 仅接受扩展数据帧，ExtId 为 (9<<8)|controller_id。
 * 数据布局为标准 VESC 格式：ERPM int32、电流 int16/10、占空比 int16/1000。
 */
void VESC_OnRx(VESC_t *vesc, const CAN_RxHeaderTypeDef *header, const uint8_t data[8], uint32_t now_ms)
{
    uint32_t erpm_raw;
    int16_t current_raw;
    int16_t duty_raw;

    if ((vesc == NULL) || (header == NULL) || (data == NULL))
    {
        return;
    }

    if ((header->IDE != CAN_ID_EXT) ||
        (header->RTR != CAN_RTR_DATA) ||
        (header->DLC != 8U) ||
        (header->ExtId !=
         ((uint32_t)(VESC_PACKET_STATUS << 8U) |
          (uint32_t)vesc->controller_id)))
    {
        return;
    }

    erpm_raw =
        ((uint32_t)data[0] << 24U) |
        ((uint32_t)data[1] << 16U) |
        ((uint32_t)data[2] << 8U) |
        (uint32_t)data[3];

    current_raw =
        (int16_t)(((uint16_t)data[4] << 8U) | data[5]);
    duty_raw =
        (int16_t)(((uint16_t)data[6] << 8U) | data[7]);

    vesc->actual_erpm = (int32_t)erpm_raw;
    vesc->actual_current_a = (float)current_raw / 10.0f;
    vesc->duty = (float)duty_raw / 1000.0f;
    vesc->last_rx_ms = now_ms;
    vesc->is_online = 1U;
}

/** @brief 保存目标 ERPM，由下一个 ControlStep 发送。 */
void VESC_SetTargetErpm(VESC_t *vesc, int32_t erpm)
{
    if (vesc != NULL)
    {
        vesc->target_erpm = erpm;
    }
}

/** @brief 使能 VESC，清除旧目标，避免使能瞬间沿用上次 ERPM。 */
void VESC_Enable(VESC_t *vesc)
{
    if (vesc == NULL)
    {
        return;
    }

    vesc->target_erpm = 0;
    vesc->is_enabled = 1U;
}

/** @brief 失能并同步发送一次 0 ERPM。 */
void VESC_Disable(VESC_t *vesc)
{
    if (vesc == NULL)
    {
        return;
    }

    vesc->target_erpm = 0;
    vesc->is_enabled = 0U;
    VESC_SendErpm(vesc, 0);
}

/**
 * @brief VESC 周期控制。
 *
 * 先检查状态帧是否超时；离线或失能时持续输出 0 ERPM，
 * 在线使能时才发送当前目标。
 */
void VESC_ControlStep(VESC_t *vesc, uint32_t now_ms)
{
    if (vesc == NULL)
    {
        return;
    }

    if (vesc->is_online &&
        ((uint32_t)(now_ms - vesc->last_rx_ms) >
         vesc->online_timeout_ms))
    {
        vesc->is_online = 0U;
    }

    if (!vesc->is_enabled || !vesc->is_online)
    {
        (void)VESC_SendErpm(vesc, 0);
        return;
    }

    VESC_SendErpm(vesc, vesc->target_erpm);
}

/**
 * @brief 打包并发送 VESC SET_RPM 扩展帧。
 *
 * ExtId=(3<<8)|controller_id，前四字节为大端 int32 ERPM，后四字节为 0。
 * last_tx_data 保存最近一次负载，串口/Watch 可据此确认编码结果。
 */
HAL_StatusTypeDef VESC_SendErpm(VESC_t *vesc, int32_t erpm)
{
    uint8_t data[8] = {0};
    uint32_t raw_erpm;
    uint32_t tx_mailbox;
    CAN_TxHeaderTypeDef header = {0};

    if (vesc == NULL)
    {
        return HAL_ERROR;
    }

    if (vesc->hcan == NULL)
    {
        vesc->last_tx_status = HAL_ERROR;
        vesc->tx_error_count++;
        return HAL_ERROR;
    }

    raw_erpm = (uint32_t)erpm;
    data[0] = (uint8_t)(raw_erpm >> 24U);
    data[1] = (uint8_t)(raw_erpm >> 16U);
    data[2] = (uint8_t)(raw_erpm >> 8U);
    data[3] = (uint8_t)raw_erpm;

    header.ExtId =
        (uint32_t)(VESC_PACKET_SET_RPM << 8U) |
        (uint32_t)vesc->controller_id;
    header.StdId = 0U;
    header.IDE = CAN_ID_EXT;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    header.TransmitGlobalTime = DISABLE;
    /* 保存最近一次发送内容，便于 Watch 和串口诊断。 */
    vesc->last_tx_id = header.ExtId;
    vesc->last_tx_erpm = erpm;

    vesc->last_tx_status =
        HAL_CAN_AddTxMessage(vesc->hcan, &header, data, &tx_mailbox);

    memcpy(vesc->last_tx_data, data, sizeof(vesc->last_tx_data));

    if (vesc->last_tx_status == HAL_OK)
    {
        vesc->tx_ok_count++;
    }
    else if (vesc->last_tx_status == HAL_BUSY)
    {
        vesc->tx_busy_count++;
    }
    else
    {
        vesc->tx_error_count++;
    }

    return vesc->last_tx_status;
}
