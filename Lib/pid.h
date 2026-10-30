#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <math.h>

/**
 * @file pid.h
 * @brief 带条件积分抗饱和和可选积分分离的位置式 PID。
 *
 * 该类 PID 只处理单个误差序列，不保存目标角或反馈角之间的业务含义。
 * 对角度等周期性量，调用者应先转换成最短误差，再传入
 * PidCalPositionError()。
 *
 * 参数单位：
 *   dt           : 秒；
 *   ki           : 输出 / (误差 * 秒)；
 *   kd           : 输出 * 秒 / 误差；
 *   output_max   : 输出绝对值限幅；
 *   integral_max : 积分项输出的绝对值限幅。
 *
 * integral 保存的是已经乘过 ki 和 dt 的积分输出，不是原始误差累加值。
 */
typedef struct Pids
{
    /* 配置参数 */
    float kp;
    float ki;
    float kd;
    float dt;
    float output_max;
    float integral_max;
    uint8_t integral_separation;
    float separation_threshold;

    /* 运行状态 */
    float target;
    float feedback;
    float error;
    float last_error;
    float output;
    float proportional;
    float integral;
    float differential;
} Pids;

/**
 * @brief 初始化 PID 参数并清空运行状态。
 * @param dt 控制周期，单位秒；非正值回退为 0.001 s。
 * @param output_max 输出绝对值限幅，内部取绝对值。
 * @param integral_max 积分项绝对值限幅，内部取绝对值。
 * @param integral_separation 非 0 时启用积分分离。
 * @param separation_threshold 积分分离阈值，内部取绝对值。
 */
void PidInit(Pids *pid, float kp, float ki, float kd, float dt, float output_max, float integral_max, uint8_t integral_separation, float separation_threshold);

/** @brief 清空误差、积分、微分和输出，不修改整定参数。 */
void PidReset(Pids *pid);

/**
 * @brief 使用已经处理好的误差执行一次位置式 PID。
 * @param error 当前误差，调用者负责完成角度解绕或目标减反馈。
 * @return 限幅后的控制输出。
 * @note 每次调用对应一个 dt；内部包含条件积分抗饱和。
 */
float PidCalPositionError(Pids *pid, float error);

/**
 * @brief 线性量的便利入口，内部计算 target-feedback。
 * @note 周期性角度不应直接使用本函数，应先通过 wrap180 求最短误差。
 */
float PidCalPosition(Pids *pid, float target, float feedback);

#endif
