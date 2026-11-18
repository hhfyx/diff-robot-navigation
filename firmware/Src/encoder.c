/**
 ******************************************************************************
 * @file    encoder.c
 * @brief   TIM2(左) / TIM3(右) 硬件编码器模式 —— 纯 CMSIS 寄存器实现
 *
 * 接线：
 *   左编码器 A -> PA0 (TIM2_CH1)   B -> PA1 (TIM2_CH2)
 *   右编码器 A -> PA6 (TIM3_CH1)   B -> PA7 (TIM2/3_CH2)
 *
 * 硬件替我们做的事：
 *   Encoder Mode 3 把 TI1/TI2 的每个边沿都计数（4 倍频），
 *   并根据两相先后自动判断方向、自动加减 CNT。
 *   CPU 唯一要做的是读 CNT，连中断都不需要。
 ******************************************************************************
 */
#include "encoder.h"
#include "board.h"
#include "gpio_ll.h"

/* TIM 寄存器位定义（避免依赖 SPL 的宏） */
#define TIM_CR1_CEN           (1u << 0)
#define TIM_CR1_ARPE          (1u << 7)
#define TIM_SMCR_SMS_Pos      0u
#define TIM_EGR_UG            (1u << 0)

/* 编码器模式 3：在 TI1 和 TI2 的所有边沿计数 */
#define TIM_SMS_ENCODER_MODE3 (3u << TIM_SMCR_SMS_Pos)

/* CCMR1: CC1S/CC2S 选输入，ICxF 配置滤波 */
#define CC1S_INPUT_TI1        (1u << 0)
#define CC2S_INPUT_TI2        (1u << 8)
#define IC1F_MAX             (0xFu << 4)
#define IC2F_MAX             (0xFu << 12)

/* 输入捕获使能 */
#define TIM_CCER_CC1E         (1u << 0)
#define TIM_CCER_CC2E         (1u << 4)

static void gpio_init(void)
{
    /* GPIOA 时钟 */
    RCC->APB2ENR |= (1u << 2);

    /* PA0/PA1 和 PA6/PA7：输入 + 上下拉（CNF=10, MODE=00 -> 0x8） */
    gpio_config_pin(ENC_L_PORT, 0u, GPIO_CONF_IN_PULL);
    gpio_config_pin(ENC_L_PORT, 1u, GPIO_CONF_IN_PULL);
    gpio_config_pin(ENC_R_PORT, 6u, GPIO_CONF_IN_PULL);
    gpio_config_pin(ENC_R_PORT, 7u, GPIO_CONF_IN_PULL);

    /* ODR 置 1 => 选上拉。内部上拉约 40kΩ，
     * 开漏输出的霍尔编码器可用它兜底；长线/强干扰请外部再加 4.7kΩ。 */
    ENC_L_PORT->BSRR = ENC_L_PINS;
    ENC_R_PORT->BSRR = ENC_R_PINS;
}

static void tim_encoder_setup(TIM_TypeDef *tim)
{
    /* 4 倍频计数，计数器自由累加到 65535 回绕 */
    tim->PSC   = 0u;
    tim->ARR   = 0xFFFFu;
    tim->CNT   = 0u;

    /* CH1 接 TI1、CH2 接 TI2，并打开最强输入滤波（抑制电刷噪声） */
    tim->CCMR1 = CC1S_INPUT_TI1 | CC2S_INPUT_TI2 | IC1F_MAX | IC2F_MAX;

    /* 两相都使能，上升沿极性（在编码器模式下极性决定计数方向） */
    tim->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E;

    /* SMS = 011：Encoder Mode 3 */
    tim->SMCR  = (tim->SMCR & ~0x7u) | TIM_SMS_ENCODER_MODE3;

    /* 产生一次更新事件把预装载值搬到影子寄存器 */
    tim->EGR  |= TIM_EGR_UG;
    tim->CR1  |= TIM_CR1_CEN;
}

void encoder_init(void)
{
    gpio_init();

    /* TIM2/TIM3 挂在 APB1 */
    RCC->APB1ENR |= (1u << 0) | (1u << 1);

    tim_encoder_setup(TIM2);   /* 左 */
    tim_encoder_setup(TIM3);   /* 右 */
}

uint16_t encoder_raw(uint8_t side)
{
    return (uint16_t)((side == 0u) ? TIM2->CNT : TIM3->CNT);
}

void encoder_zero(void)
{
    TIM2->CNT = 0u;
    TIM3->CNT = 0u;
}
