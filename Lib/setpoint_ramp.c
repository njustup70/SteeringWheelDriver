#include "setpoint_ramp.h"

#include <math.h>
#include <stddef.h>

/** @brief 将角度归一化到 [0, 360)。 */
static float SetpointRamp_Wrap360(float angle_deg)
{
    angle_deg = fmodf(angle_deg, 360.0f);
    if (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

/** @brief 将角度归一化到 [-180, 180)。 */
static float SetpointRamp_Wrap180(float angle_deg)
{
    angle_deg = SetpointRamp_Wrap360(angle_deg);
    if (angle_deg >= 180.0f)
    {
        angle_deg -= 360.0f;
    }
    return angle_deg;
}

/** @brief 将增量限制在 [-limit, +limit]。 */
static float SetpointRamp_Clamp(float value, float limit)
{
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
 * @brief 初始化斜坡。
 * @note 非正速度和加速度会回退到头文件默认值，避免除零或零速度死锁。
 */
void SetpointRamp_Init(SetpointRamp_t *ramp,
                       float initial_angle_deg,
                       float max_speed_deg_s,
                       float max_accel_deg_s2)
{
    if (ramp == NULL)
    {
        return;
    }

    ramp->max_speed_deg_s = fabsf(max_speed_deg_s);
    if (ramp->max_speed_deg_s <= 0.0f)
    {
        ramp->max_speed_deg_s =
            SETPOINT_RAMP_DEFAULT_MAX_SPEED_DEG_S;
    }

    ramp->max_accel_deg_s2 = fabsf(max_accel_deg_s2);
    if (ramp->max_accel_deg_s2 <= 0.0f)
    {
        ramp->max_accel_deg_s2 =
            SETPOINT_RAMP_DEFAULT_MAX_ACCEL_DEG_S2;
    }

    ramp->desired_angle_deg =
        SetpointRamp_Wrap360(initial_angle_deg);
    ramp->output_angle_deg = ramp->desired_angle_deg;
    ramp->velocity_deg_s = 0.0f;
}

/** @brief 把目标与输出同步到当前角，适合首次使能或故障恢复。 */
void SetpointRamp_Reset(SetpointRamp_t *ramp,
                        float current_angle_deg)
{
    if (ramp == NULL)
    {
        return;
    }

    ramp->output_angle_deg =
        SetpointRamp_Wrap360(current_angle_deg);
    ramp->desired_angle_deg = ramp->output_angle_deg;
    ramp->velocity_deg_s = 0.0f;
}

/** @brief 只改变最终目标，不打断正在进行的斜坡。 */
void SetpointRamp_SetDesired(SetpointRamp_t *ramp,
                             float desired_angle_deg)
{
    if (ramp != NULL)
    {
        ramp->desired_angle_deg =
            SetpointRamp_Wrap360(desired_angle_deg);
    }
}

/**
 * @brief 推进一次位置参考斜坡。
 *
 * sqrt(2*A*error) 是理论上按最大减速度恰好停在目标处允许的速度包络；
 * 再与最大速度取小得到本周期目标速度。速度本身按 A*dt 限幅变化，
 * 因而输出轨迹同时满足速度上限和加速度上限。
 */
void SetpointRamp_Update(SetpointRamp_t *ramp, float dt_s)
{
    float error_deg;
    float target_speed;
    float speed_magnitude;
    float max_delta_speed;
    float speed_delta;

    if ((ramp == NULL) || (dt_s <= 0.0f))
    {
        return;
    }

    error_deg = SetpointRamp_Wrap180(
        ramp->desired_angle_deg - ramp->output_angle_deg);

    if ((fabsf(error_deg) <= SETPOINT_RAMP_ANGLE_EPS_DEG) &&
        (fabsf(ramp->velocity_deg_s) <=
         SETPOINT_RAMP_VEL_EPS_DEG_S))
    {
        ramp->output_angle_deg = ramp->desired_angle_deg;
        ramp->velocity_deg_s = 0.0f;
        return;
    }

    /* 根据剩余角距离计算允许的制动速度包络。 */
    speed_magnitude = sqrtf(
        2.0f * ramp->max_accel_deg_s2 * fabsf(error_deg));
    if (speed_magnitude > ramp->max_speed_deg_s)
    {
        speed_magnitude = ramp->max_speed_deg_s;
    }

    target_speed = (error_deg > 0.0f)
                       ? speed_magnitude
                       : -speed_magnitude;

    /* 每周期允许的参考速度增量，体现加速度限制。 */
    max_delta_speed = ramp->max_accel_deg_s2 * dt_s;
    speed_delta = SetpointRamp_Clamp(
        target_speed - ramp->velocity_deg_s,
        max_delta_speed);
    ramp->velocity_deg_s += speed_delta;

    ramp->output_angle_deg = SetpointRamp_Wrap360(
        ramp->output_angle_deg + ramp->velocity_deg_s * dt_s);

}

/** @brief 返回当前斜坡输出；空指针时返回 0。 */
float SetpointRamp_GetOutput(const SetpointRamp_t *ramp)
{
    return (ramp == NULL) ? 0.0f : ramp->output_angle_deg;
}

/** @brief 返回当前参考角速度；空指针时返回 0。 */
float SetpointRamp_GetVelocity(const SetpointRamp_t *ramp)
{
    return (ramp == NULL) ? 0.0f : ramp->velocity_deg_s;
}
