#ifndef MASTER_CAN_H
#define MASTER_CAN_H

#include <stdint.h>

#include "main.h"

/**
 * @file MasterCan.h
 * @brief 主控标准帧协议解码和链路状态管理。
 *
 * 主控在目标变化时发送控制帧，并周期发送心跳帧。
 * 控制目标在链路正常期间持续有效；链路超时后 control_seen 被清除，
 * 恢复心跳也不能恢复旧目标，必须等待新的控制帧。
 */

#define MASTER_CONTROL_ID_BASE     0x100U
#define MASTER_HEARTBEAT_ID_BASE   0x110U
#define MASTER_LINK_TIMEOUT_MS     500U

typedef struct
{
    /* 轮号和协议 ID */
    uint8_t wheel_id;
    uint32_t control_id;
    uint32_t heartbeat_id;

    /* 主控命令 */
    int32_t drive_erpm;
    uint16_t angle_code;
    float target_angle_deg;

    /* 时间戳与链路超时 */
    uint32_t last_control_ms;
    uint32_t last_heartbeat_ms;
    uint32_t link_timeout_ms;

    /* 链路状态 */
    uint8_t control_seen;
    uint8_t heartbeat_seen;
    uint8_t link_valid;
} MasterCan_t;

/** @brief 初始化轮号、协议 ID 和链路状态。 */
void MasterCan_Init(MasterCan_t *master, uint8_t wheel_id);

/** @brief 解析本轮控制帧或心跳帧。 */
void MasterCan_OnRx(MasterCan_t *master,
                    const CAN_RxHeaderTypeDef *header,
                    const uint8_t data[8],
                    uint32_t now_ms);

/**
 * @brief 更新 link_valid，并在链路失效时清除 control_seen。
 * @note 必须在周期任务中调用；心跳处理前也会调用一次以检测链路中断。
 */
void MasterCan_Update(MasterCan_t *master, uint32_t now_ms);

#endif /* MASTER_CAN_H */
