/**
 ******************************************************************************
 * @file    motor.h
 * @brief   TB6612 / DRV8833 两路直流电机驱动封装
 *
 * 对外只暴露"带符号占空比"这一个接口：正值正转，负值反转，0 = 惰行。
 * 内部负责：方向脚切换、死区补偿、单侧限幅。
 ******************************************************************************
 */
#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

/** 给定 -PWM_ARR ~ +PWM_ARR 的带符号占空比，左轮 */
void motor_set_left(int16_t duty);
/** 同上，右轮 */
void motor_set_right(int16_t duty);

/** 立即刹车（两相短路，H 桥两端同时拉低/拉高） */
void motor_brake_all(void);
/** 惰行停车（占空比归零 + STBY 拉低） */
void motor_coast_all(void);
/** 初始化 GPIO 与 TIM4 PWM，STBY 拉高使能 */
void motor_init(void);

#endif /* __MOTOR_H */
