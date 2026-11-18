/**
 ******************************************************************************
 * @file    pid.c
 * @brief   增量式 PID
 *
 * 增量式公式：
 *   Δu(k) = Kp·[e(k) − e(k−1)]
 *         + Ki·dt·e(k)
 *         + Kd/dt·[e(k) − 2e(k−1) + e(k−2)]
 *   u(k)  = u(k−1) + Δu(k)，再做 ±output_limit 限幅
 *
 * 增益整定建议（README 有完整流程）：
 *   1) Ki=Kd=0，Kp 从小加到"等幅振荡"，记 Kp_osc；
 *   2) Kp = 0.5·Kp_osc 起步；
 *   3) 加 Ki 消稳态误差，Ki 从 0.02 起，每次翻倍直到 2 s 内收敛；
 *   4) Kd 通常给 0~0.5·Kp，编码器噪声大时干脆给 0。
 ******************************************************************************
 */
#include "pid.h"

void PID_Init(PID_t *p, float kp, float ki, float kd, float out_limit)
{
    if (p == 0) return;

    p->Kp = kp;
    p->Ki = ki;
    p->Kd = kd;

    p->target      = 0.0f;
    p->feedback    = 0.0f;
    p->error       = 0.0f;
    p->last_error  = 0.0f;
    p->prev_error  = 0.0f;
    p->integral    = 0.0f;
    p->derivative  = 0.0f;
    p->output      = 0.0f;
    p->output_limit = out_limit;
}

void PID_Reset(PID_t *p)
{
    if (p == 0) return;

    p->target     = 0.0f;
    p->feedback   = 0.0f;
    p->error      = 0.0f;
    p->last_error = 0.0f;
    p->prev_error = 0.0f;
    p->integral   = 0.0f;
    p->derivative = 0.0f;
    p->output     = 0.0f;
}

float PID_UpdateIncremental(PID_t *p, float target, float feedback, float dt)
{
    float delta;
    float out;

    if (p == 0) {
        return 0.0f;
    }
    /* dt 异常保护：定时器配置错时不要让 PID 输出变成 inf */
    if (dt <= 0.0f) {
        return p->output;
    }

    p->target   = target;
    p->feedback = feedback;
    p->error    = target - feedback;

    /* 增量式三项 */
    delta  = p->Kp * (p->error - p->last_error);
    delta += p->Ki * dt * p->error;
    delta += (p->Kd / dt) * (p->error - 2.0f * p->last_error + p->prev_error);

    p->derivative = (p->error - p->last_error) / dt;

    /* 历史偏差滚动 */
    p->prev_error = p->last_error;
    p->last_error = p->error;

    out = p->output + delta;

    /* 对称限幅 + 抗积分饱和：限幅时把累计输出钳住，
     * 避免长时间顶死后退回时"粘"在限幅值上 */
    if (out > p->output_limit) {
        out = p->output_limit;
    } else if (out < -p->output_limit) {
        out = -p->output_limit;
    }

    p->output = out;
    return out;
}
