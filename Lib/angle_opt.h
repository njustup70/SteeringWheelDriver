#ifndef ANGLE_OPT_H
#define ANGLE_OPT_H

#include <stdint.h>

/**
 * @file angle_opt.h
 * @brief 舵轮物理等价方向选择。
 *
 * 对于目标 `(angle, erpm)`，`(angle+180, -erpm)` 对应同一个底盘运动方向。
 * 本模块选择与当前舵向角距离更短的一组，从而减少舵向旋转量。
 */

typedef struct
{
    /* 最终舵向角，范围 [0, 360)。 */
    float target_angle_deg;
    /* 最终 ERPM，反向方案时符号取反。 */
    int32_t target_erpm;
    /* 1 表示选择了反向物理等效方案。 */
    uint8_t reversed;
} AngleOptResult_t;

/**
 * @brief 在原始目标与反向等价目标之间选择更近的一组。
 * @param current_angle_deg 当前舵轮本地坐标角。
 * @param target_angle_deg 主控坐标映射后的目标角。
 * @param target_erpm 主控 ERPM。
 * @param result 输出结果；为空时不写任何数据。
 */
void AngleOpt_Select(float current_angle_deg,
                     float target_angle_deg,
                     int32_t target_erpm,
                     AngleOptResult_t *result);

#endif /* ANGLE_OPT_H */
