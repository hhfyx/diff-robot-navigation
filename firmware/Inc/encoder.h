/**
 ******************************************************************************
 * @file    encoder.h
 * @brief   TIM2/TIM3 硬件编码器模式驱动
 *
 * 只做最原始的事：把 AB 相接到定时器 CH1/CH2，配置成 Encoder Mode 3
 * （两相上下沿都计数 = 4 倍频），鉴相和累加全部由硬件完成，CPU 只读 CNT。
 ******************************************************************************
 */
#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

/** 初始化 TIM2(左) / TIM3(右) 为编码器模式，计数器清零 */
void encoder_init(void);

/** 读取原始计数（0~65535，会自动回绕） */
uint16_t encoder_raw(uint8_t side);   /* side: 0=左, 1=右 */

/** 清空计数器（标定/归零用） */
void encoder_zero(void);

#endif /* __ENCODER_H */
