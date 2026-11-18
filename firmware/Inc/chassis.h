/**
 ******************************************************************************
 * @file    chassis.h
 * @brief   底盘运动控制：编码器测速 + 斜坡 + 双轮独立 PID
 *
 * 设计要点：
 *   - 控制节拍固定 1 kHz（SysTick），与串口收发完全解耦；
 *   - 测速窗口 10 ms（滑窗），既保证分辨率又保证更新率；
 *   - 目标值先过斜坡发生器再进 PID，避免阶跃冲击与编码器噪声放大；
 *   - 底盘级速度上限按 |v| + |w|*B/2 缩放，保证转弯时不超单轮限幅。
 ******************************************************************************
 */
#ifndef __CHASSIS_H
#define __CHASSIS_H

#include <stdint.h>

/** 初始化：编码器、电机、左右两套 PID 一次配好 */
void chassis_init(void);

/** 1 kHz 节拍调用（SysTick 中断里调），完成测速 + 斜坡 + PID + PWM 输出 */
void chassis_tick(void);

/**
 * 设置目标运动（上位机指令入口）
 * @param v  线速度 [m/s]，正=前进
 * @param w  角速度 [rad/s]，正=左转（逆时针，右手定则）
 */
void chassis_set_cmd(float v, float w);

/** 立即停车（安全层/急停/断链调用），清 PID 与斜坡状态 */
void chassis_stop(void);

/** 是否处于链路超时保护中 */
uint8_t chassis_is_link_lost(void);

/* ---------- 供调试/上位机读取的快照 ---------- */
typedef struct {
    float v_left;        /* 左轮线速度 [m/s] */
    float v_right;       /* 右轮线速度 [m/s] */
    float v_target;      /* 斜坡后的左轮目标 [m/s] */
    float w_target;      /* 斜坡后的右轮目标 [m/s] */
    float v;             /* 底盘线速度 [m/s] */
    float w;             /* 底盘角速度 [rad/s] */
    int32_t ticks_left;  /* 左轮累计脉冲（有符号，可溢出回绕） */
    int32_t ticks_right; /* 右轮累计脉冲 */
    int16_t duty_left;   /* 左轮实际输出占空比 */
    int16_t duty_right;  /* 右轮实际输出占空比 */
    uint8_t link_lost;   /* 1 = 链路超时已自动停车 */
} ChassisState_t;

/** 取当前快照（拷贝，非指针，避免中断竞态） */
ChassisState_t chassis_snapshot(void);

#endif /* __CHASSIS_H */
