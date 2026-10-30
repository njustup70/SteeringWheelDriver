#include "bsp_can.h"

/**
 * @brief 获取指定 CAN 外设可用过滤器组的起始 bank。
 * @note STM32F4 的 CAN2 共享过滤器组从 bank 14 开始。
 */
static uint32_t BSP_CAN_FirstFilterBank(const CAN_HandleTypeDef *hcan)
{
    return (hcan->Instance == CAN2) ? 14U : 0U;
}

/**
 * @brief 配置一个精确匹配 11 位标准 ID 的过滤器。
 *
 * bxCAN 的 32 位过滤器把标准 ID 放在高寄存器的 bit[31:21]，
 * 因此标准 ID 需要左移 5 位。掩码覆盖全部 11 位，实现精确匹配。
 */
static HAL_StatusTypeDef BSP_CAN_ConfigureStdFilter(
    CAN_HandleTypeDef *hcan,
    uint32_t filter_bank,
    uint32_t std_id)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterBank = filter_bank;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = (uint16_t)(std_id << 5U);
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = (uint16_t)(0x7FFU << 5U);
    filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;

    return HAL_CAN_ConfigFilter(hcan, &filter);
}

/**
 * @brief 按 HAL 要求顺序启动 CAN 并开启 FIFO0 接收通知。
 */
static HAL_StatusTypeDef BSP_CAN_StartWithNotification(
    CAN_HandleTypeDef *hcan)
{
    HAL_StatusTypeDef status = HAL_CAN_Start(hcan);

    if (status != HAL_OK)
    {
        return status;
    }

    return HAL_CAN_ActivateNotification(
        hcan,
        CAN_IT_RX_FIFO0_MSG_PENDING);
}

HAL_StatusTypeDef BSP_CAN_StartAcceptAll(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef filter = {0};
    HAL_StatusTypeDef status;

    if (hcan == NULL)
    {
        return HAL_ERROR;
    }

    /* ID 和掩码全部为 0：任意标准帧、扩展帧均可进入 FIFO0。 */
    filter.FilterBank = BSP_CAN_FirstFilterBank(hcan);
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;

    status = HAL_CAN_ConfigFilter(hcan, &filter);
    if (status != HAL_OK)
    {
        return status;
    }

    return BSP_CAN_StartWithNotification(hcan);
}

HAL_StatusTypeDef BSP_CAN_StartMasterFilter(
    CAN_HandleTypeDef *hcan,
    uint8_t wheel_id)
{
    uint32_t first_bank;
    uint32_t control_id;
    uint32_t heartbeat_id;
    HAL_StatusTypeDef status;

    if ((hcan == NULL) || (wheel_id < 1U) || (wheel_id > 4U))
    {
        return HAL_ERROR;
    }

    /* 使用相邻两个过滤器组分别精确匹配控制帧和心跳帧。 */
    first_bank = BSP_CAN_FirstFilterBank(hcan);
    control_id = 0x100U + wheel_id;
    heartbeat_id = 0x110U + wheel_id;

    status = BSP_CAN_ConfigureStdFilter(
        hcan,
        first_bank,
        control_id);
    if (status != HAL_OK)
    {
        return status;
    }

    status = BSP_CAN_ConfigureStdFilter(
        hcan,
        first_bank + 1U,
        heartbeat_id);
    if (status != HAL_OK)
    {
        return status;
    }

    return BSP_CAN_StartWithNotification(hcan);
}
