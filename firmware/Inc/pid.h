/**
 ******************************************************************************
 * @file    pid.h
 * @brief   增量式 PID —— 速度环用
 *
 * 为什么用增量式：输出是"上一拍输出 + Δu"，天然带积分记忆，
 * 而且换目标时不会像位置式那样累积巨大积分产生超调。
 ******************************************************************************
 */
#ifndef __PID_H
#define __PID_H

typedef struct {
    /* --- 参数（可在线修改，便于串口调参） --- */
    float Kp;
    float Ki;
    float Kd;

    /* --- 运行状态 --- */
    float target;        /* 目标值 */
    float feedback;      /* 实际反馈值 */
    float error;         /* 当前偏差 */
    float last_error;    /* 上一拍偏差 */
    float prev_error;    /* 上上拍偏差 */
    float integral;      /* 位置式用的积分项（保留，便于扩展） */
    float derivative;    /* 微分项 */
    float output;        /* 累计输出 */
    float output_limit;  /* 输出限幅（对称 ±） */
} PID_t;

/** 初始化：把参数与状态一次写全 */
void PID_Init(PID_t *p, float kp, float ki, float kd, float out_limit);

/** 清零历史状态（切换工作模式、急停后调用），保留 Kp/Ki/Kd */
void PID_Reset(PID_t *p);

/**
 * 增量式 PID 单步计算
 * @param p        控制器实例（左右轮各一个，严禁共用一个）
 * @param target   目标值
 * @param feedback 反馈值
 * @param dt       控制周期 [s]
 * @return 限幅后的累计输出
 */
float PID_UpdateIncremental(PID_t *p, float target, float feedback, float dt);

#endif /* __PID_H */
