#ifndef SETPOINT_RAMP_H
#define SETPOINT_RAMP_H

#include <stdint.h>

/**
 * @file setpoint_ramp.h
 * @brief 带速度上限和加速度上限的角位置参考斜坡。
 *
 * 输出是送给位置环的“移动目标角”，不是实际电机角。
 * 速度单位为 °/s，加速度单位为 °/s²。
 */

#define SETPOINT_RAMP_DEFAULT_MAX_SPEED_DEG_S   790.0f
#define SETPOINT_RAMP_DEFAULT_MAX_ACCEL_DEG_S2  4000.0f
#define SETPOINT_RAMP_ANGLE_EPS_DEG             0.05f
#define SETPOINT_RAMP_VEL_EPS_DEG_S             0.5f

typedef struct
{
    /* 原始目标角和当前斜坡输出角，范围均为 [0, 360)。 */
    float desired_angle_deg;
    float output_angle_deg;
    /* 当前参考角速度，单位 °/s。 */
    float velocity_deg_s;
    /* 运行时限制。 */
    float max_speed_deg_s;
    float max_accel_deg_s2;
} SetpointRamp_t;

/** @brief 初始化斜坡起点、速度上限和加速度上限。 */
void SetpointRamp_Init(SetpointRamp_t *ramp,
                       float initial_angle_deg,
                       float max_speed_deg_s,
                       float max_accel_deg_s2);

/** @brief 把斜坡输出重置到当前实际角，同时清零速度。 */
void SetpointRamp_Reset(SetpointRamp_t *ramp,
                        float current_angle_deg);

/** @brief 设置新的最终目标角，不重置当前斜坡状态。 */
void SetpointRamp_SetDesired(SetpointRamp_t *ramp,
                             float desired_angle_deg);

/** @brief 使用给定控制周期推进一次斜坡。 */
void SetpointRamp_Update(SetpointRamp_t *ramp, float dt_s);

/** @brief 获取当前斜坡输出角。 */
float SetpointRamp_GetOutput(const SetpointRamp_t *ramp);
/** @brief 获取当前参考角速度。 */
float SetpointRamp_GetVelocity(const SetpointRamp_t *ramp);

#endif /* SETPOINT_RAMP_H */
