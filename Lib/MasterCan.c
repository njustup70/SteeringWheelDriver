#include "MasterCan.h"

/** @brief 初始化轮号、ID、命令字段和链路状态。 */
void MasterCan_Init(MasterCan_t *master, uint8_t wheel_id)
{
    if ((master == NULL) || (wheel_id < 1U) || (wheel_id > 4U))
    {
        Error_Handler();
        return;
    }

    master->wheel_id = wheel_id;
    master->control_id = MASTER_CONTROL_ID_BASE + wheel_id;
    master->heartbeat_id = MASTER_HEARTBEAT_ID_BASE + wheel_id;

    master->drive_erpm = 0;
    master->angle_code = 0U;
    master->target_angle_deg = 0.0f;

    master->last_control_ms = 0U;
    master->last_heartbeat_ms = 0U;
    master->link_timeout_ms = MASTER_LINK_TIMEOUT_MS;

    master->control_seen = 0U;
    master->heartbeat_seen = 0U;
    master->link_valid = 0U;
}

/**
 * @brief 解码主控控制帧或心跳帧。
 *
 * 控制帧 Data[0..3] 为大端 int32 ERPM，Data[4..7] 为大端角度码。
 * 角度码越界时夹紧到 0..16383。控制帧同时刷新控制时间戳和心跳时间戳。
 */
void MasterCan_OnRx(MasterCan_t *master,
                    const CAN_RxHeaderTypeDef *header,
                    const uint8_t data[8],
                    uint32_t now_ms)
{
    uint32_t erpm_raw;
    int32_t angle_raw;

    if ((master == NULL) || (header == NULL) || (data == NULL))
    {
        return;
    }

    if ((header->IDE != CAN_ID_STD) ||
        (header->RTR != CAN_RTR_DATA) ||
        (header->DLC != 8U))
    {
        return;
    }

    if (header->StdId == master->control_id)
    {
        erpm_raw =
            ((uint32_t)data[0] << 24U) |
            ((uint32_t)data[1] << 16U) |
            ((uint32_t)data[2] << 8U) |
            (uint32_t)data[3];

        angle_raw =
            (int32_t)(((uint32_t)data[4] << 24U) |
                      ((uint32_t)data[5] << 16U) |
                      ((uint32_t)data[6] << 8U) |
                      (uint32_t)data[7]);

        if (angle_raw < 0)
        {
            angle_raw = 0;
        }
        else if (angle_raw > 16383)
        {
            angle_raw = 16383;
        }

        master->drive_erpm = (int32_t)erpm_raw;
        master->angle_code = (uint16_t)angle_raw;
        master->target_angle_deg =
            (float)master->angle_code * (360.0f / 16384.0f);

        master->last_control_ms = now_ms;
        master->last_heartbeat_ms = now_ms;
        master->control_seen = 1U;
        master->heartbeat_seen = 1U;

        MasterCan_Update(master, now_ms);
    }
    else if (header->StdId == master->heartbeat_id)
    {
        /*
         * 先按旧时间戳检查一次链路。若已经超时，
         * control_seen 会被清除，恢复心跳也不会复活旧目标。
         */
        MasterCan_Update(master, now_ms);

        master->last_heartbeat_ms = now_ms;
        master->heartbeat_seen = 1U;

        MasterCan_Update(master, now_ms);
    }
}

/**
 * @brief 根据最长 500 ms 时间窗更新主控链路状态。
 *
 * 控制帧或心跳任一仍在有效期内即可维持 link_valid。
 * 链路失效时清除 control_seen，但命令数值保留用于诊断。
 * 所有时间差使用 uint32_t 无符号减法，天然支持毫秒计数回绕。
 */
void MasterCan_Update(MasterCan_t *master, uint32_t now_ms)
{
    uint8_t link_valid = 0U;

    if (master == NULL)
    {
        return;
    }

    if (master->control_seen)
    {
        if ((uint32_t)(now_ms - master->last_control_ms) <=
            master->link_timeout_ms)
        {
            link_valid = 1U;
        }
    }

    if (master->heartbeat_seen)
    {
        if ((uint32_t)(now_ms - master->last_heartbeat_ms) <=
            master->link_timeout_ms)
        {
            link_valid = 1U;
        }
    }

    /*
     * 链路失效后清除控制可用标志，但保留最后命令值用于诊断。
     * 下一次心跳只能恢复 link_valid，不能直接恢复旧目标。
     */
    master->link_valid = link_valid;
    if (!link_valid)
    {
        master->control_seen = 0U;
    }
}
