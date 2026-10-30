#include "angle_opt.h"

#include <math.h>
#include <stddef.h>

/** @brief 将角度归一化到 [0, 360)。 */
static float AngleOpt_Wrap360(float angle_deg)
{
    angle_deg = fmodf(angle_deg, 360.0f);
    if (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

/** @brief 将角度归一化到 [-180, 180)，表示最短角距离。 */
static float AngleOpt_Wrap180(float angle_deg)
{
    angle_deg = AngleOpt_Wrap360(angle_deg);
    if (angle_deg >= 180.0f)
    {
        angle_deg -= 360.0f;
    }
    return angle_deg;
}

/**
 * @brief 从两个物理等价方向中选出离当前舵向更近的方案。
 *
 * A=(target, erpm)，B=(target+180, -erpm)。
 * 距离相等时保留 A，避免 ERPM 无必要翻转。
 * ERPM 为 0 时不构造反向方案，只把目标角转到归一化值。
 */
void AngleOpt_Select(float current_angle_deg,
                     float target_angle_deg,
                     int32_t target_erpm,
                     AngleOptResult_t *result)
{
    float current_norm;
    float target_norm;
    float reversed_angle;
    float error_a;
    float error_b;

    if (result == NULL)
    {
        return;
    }

    current_norm = AngleOpt_Wrap360(current_angle_deg);
    target_norm = AngleOpt_Wrap360(target_angle_deg);

    result->target_angle_deg = target_norm;
    result->target_erpm = target_erpm;
    result->reversed = 0U;

    if (target_erpm == 0)
    {
        return;
    }

    reversed_angle = AngleOpt_Wrap360(target_norm + 180.0f);

    error_a = AngleOpt_Wrap180(target_norm - current_norm);
    error_b = AngleOpt_Wrap180(reversed_angle - current_norm);

    /* 严格小于保证平局时优先不反转 ERPM。 */
    if (fabsf(error_b) < fabsf(error_a))
    {
        result->target_angle_deg = reversed_angle;
        result->target_erpm = -target_erpm;
        result->reversed = 1U;
    }
}
