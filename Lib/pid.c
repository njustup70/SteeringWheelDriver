#include "pid.h"

/** @brief 将数值限制在 [-limit, +limit]；非正 limit 返回 0。 */
static float PidClampAbs(float value, float limit)
{
    if (limit <= 0.0f)
    {
        return 0.0f;
    }

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
 * @brief 初始化配置参数并复位运行状态。
 * @note ki/kd 的物理量纲由调用者保持一致；本模块不自动换算单位。
 */
void PidInit(Pids *pid, float kp, float ki, float kd, float dt, float output_max, float integral_max, uint8_t integral_separation, float separation_threshold)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = (dt > 0.0f) ? dt : 0.001f;
    pid->output_max = fabsf(output_max);
    pid->integral_max = fabsf(integral_max);
    pid->integral_separation = integral_separation;
    pid->separation_threshold = fabsf(separation_threshold);

    PidReset(pid);
}

/** @brief 只清空运行时状态，保留 kp/ki/kd、dt 和限幅配置。 */
void PidReset(Pids *pid)
{
    pid->target = 0.0f;
    pid->feedback = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->output = 0.0f;
    pid->proportional = 0.0f;
    pid->integral = 0.0f;
    pid->differential = 0.0f;
}

/**
 * @brief 执行一次位置式 PID 计算。
 *
 * 计算顺序：
 * 1. 比例项和误差差分项；
 * 2. 根据积分分离条件决定是否累加积分；
 * 3. 计算候选总输出；
 * 4. 若输出已饱和且误差继续推向饱和方向，则回滚本次积分增量；
 * 5. 记录本次误差并返回限幅输出。
 *
 * @note last_error 在每次调用尾部更新，因此 Kd 使用的是本周期与上周期误差差。
 */
float PidCalPositionError(Pids *pid, float error)
{
    float integral_next = pid->integral;
    uint8_t integrate = 1U;

    pid->proportional = pid->kp * error;

    pid->differential = 0.0f;
    if (pid->kd != 0.0f)
    {
        pid->differential =
            pid->kd * (error - pid->last_error) / pid->dt;
    }

    if (pid->integral_separation &&
        (fabsf(error) > pid->separation_threshold))
    {
        /* 积分分离：大误差阶段主要依靠比例项，避免积分快速堆积。 */
        integrate = 0U;
    }

    if (integrate)
    {
        integral_next += pid->ki * error * pid->dt;
        integral_next = PidClampAbs(integral_next, pid->integral_max);
    }

    pid->output =
        pid->proportional + integral_next + pid->differential;

    /*
     * 条件积分抗饱和：
     * 只有“误差继续推向已经饱和的方向”时才回滚积分。若误差已经反向，
     * 允许积分继续变化，以便控制器能够退出饱和。
     */
    if (((pid->output > pid->output_max) && (error > 0.0f)) ||
        ((pid->output < -pid->output_max) && (error < 0.0f)))
    {
        integral_next = pid->integral;
        pid->output =
            pid->proportional + integral_next + pid->differential;
    }

    pid->integral = integral_next;
    pid->error = error;
    pid->last_error = error;
    pid->output = PidClampAbs(pid->output, pid->output_max);

    return pid->output;
}

/**
 * @brief 线性量 PID 便利入口。
 * @note 对角度误差请先调用业务角解绕函数，再使用 PidCalPositionError。
 */
float PidCalPosition(Pids *pid, float target, float feedback)
{
    pid->target = target;
    pid->feedback = feedback;

    return PidCalPositionError(pid, target - feedback);
}
