/**
 ******************************************************************************
 * @file    stm32f10x.h
 * @brief   STM32F103C8T6 最小自包含寄存器定义（不依赖 ST 标准外设库）
 *
 * 为什么不用 SPL：
 *   SPL 有一百多个源文件，新手配环境常常卡一整天；
 *   本工程只用 GPIO / TIM / USART / RCC 四种外设，把用到的寄存器
 *   直接写清楚反而更透明，也更容易看清"每个位到底在干什么"。
 *
 * 依赖：只有 arm-none-eabi-gcc + 本头文件，无任何外部库。
 ******************************************************************************
 */
#ifndef __STM32F10X_H
#define __STM32F10X_H

#include <stdint.h>

/*==============================================================================
 * 1. 基础类型
 *============================================================================*/
typedef enum { RESET = 0, SET = 1 } FlagStatus, ITStatus;
typedef enum { DISABLE = 0, ENABLE = 1 } FunctionalState;

#define __IO    volatile
#define __I     volatile const
#define __O     volatile

#define __NVIC_PRIO_BITS   4

/* GCC 汇编关键字。写成 __asm__ 而不是 asm，
 * 是为了在 -std=c99 -pedantic 下也不报警告。 */
#define __NOP()     __asm__ __volatile__ ("nop" ::: "memory")
#define __DSB()     __asm__ __volatile__ ("dsb" ::: "memory")
#define __ISB()     __asm__ __volatile__ ("isb" ::: "memory")
#define __WFI()     __asm__ __volatile__ ("wfi" ::: "memory")

/*==============================================================================
 * 2. Cortex-M3 内核外设
 *============================================================================*/

typedef struct {
    __IO uint32_t CTRL;
    __IO uint32_t LOAD;
    __IO uint32_t VAL;
    __I  uint32_t CALIB;
} SysTick_Type;

#define SysTick                 ((SysTick_Type *)0xE000E010u)
#define SysTick_CTRL_ENABLE     (1u << 0)
#define SysTick_CTRL_TICKINT    (1u << 1)
#define SysTick_CTRL_CLKSOURCE  (1u << 2)
#define SysTick_CTRL_COUNTFLAG  (1u << 16)

/* NVIC 使能寄存器与优先级寄存器 */
#define NVIC_ISER0              (*(__IO uint32_t *)0xE000E100u)
#define NVIC_ISER1              (*(__IO uint32_t *)0xE000E104u)
#define NVIC_IPR                ((__IO uint8_t  *)0xE000E400u)

/* 系统异常优先级寄存器（SCB->SHP），一共 12 个字节，
 * 依次对应异常号 4(SVCall) ... 15(SysTick)。 */
#define SCB_SHP                 ((__IO uint8_t  *)0xE000ED18u)

#define SysTick_IRQn            (-1)        /* 内核异常，遵循 CMSIS 约定 */
#define USART1_IRQn             37

/**
 * 设置中断优先级。
 * @param irqn 中断号。SysTick 用 -1（CMSIS 约定），外部中断用其 IRQn
 * @param prio 抢占优先级 0~15，数值越小越高
 */
static inline void NVIC_SetPriority(int32_t irqn, uint32_t prio)
{
    uint8_t v = (uint8_t)((prio & 0xFu) << (8u - __NVIC_PRIO_BITS));

    if (irqn >= 0) {
        NVIC_IPR[irqn] = v;
    } else {
        /* CMSIS: SHP[(irq & 0xF) - 4]。SysTick(-1) => (15-4)=11 => 0xE000ED23 */
        SCB_SHP[((uint32_t)(irqn) & 0xFu) - 4u] = v;
    }
}

static inline void NVIC_EnableIRQ(int32_t irqn)
{
    if (irqn >= 32) {
        NVIC_ISER1 = (1u << ((uint32_t)irqn - 32u));
    } else if (irqn >= 0) {
        NVIC_ISER0 = (1u << (uint32_t)irqn);
    }
}

/**
 * 配置 SysTick，使其每 ticks 个内核时钟中断一次。
 * 与 CMSIS 的 SysTick_Config 行为一致。
 * @return 0 成功，1 失败（ticks 超出 24 位）
 */
static inline uint32_t SysTick_Config(uint32_t ticks)
{
    if ((ticks - 1u) > 0xFFFFFFu) {
        return 1u;
    }
    SysTick->LOAD = ticks - 1u;
    NVIC_SetPriority(SysTick_IRQn, 1u);      /* 默认抢占 1，高于串口的 2 */
    SysTick->VAL  = 0u;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE |
                    SysTick_CTRL_TICKINT |
                    SysTick_CTRL_ENABLE;
    return 0u;
}

static inline void __disable_irq(void) { __asm__ __volatile__ ("cpsid i" ::: "memory"); }
static inline void __enable_irq(void)  { __asm__ __volatile__ ("cpsie i" ::: "memory"); }

/*==============================================================================
 * 3. 外设寄存器结构体
 *============================================================================*/

typedef struct {
    __IO uint32_t CRL;      /* 0x00 引脚 0~7  配置 */
    __IO uint32_t CRH;      /* 0x04 引脚 8~15 配置 */
    __IO uint32_t IDR;      /* 0x08 输入数据 */
    __IO uint32_t ODR;      /* 0x0C 输出数据 */
    __IO uint32_t BSRR;     /* 0x10 置位/复位，写 1 有效 */
    __IO uint32_t BRR;      /* 0x14 只复位 */
    __IO uint32_t LCKR;     /* 0x18 配置锁定 */
} GPIO_TypeDef;

typedef struct {
    __IO uint32_t CR1;      /* 0x00 */
    __IO uint32_t CR2;      /* 0x04 */
    __IO uint32_t SMCR;     /* 0x08 */
    __IO uint32_t DIER;     /* 0x0C */
    __IO uint32_t SR;       /* 0x10 */
    __IO uint32_t EGR;      /* 0x14 */
    __IO uint32_t CCMR1;    /* 0x18 */
    __IO uint32_t CCMR2;    /* 0x1C */
    __IO uint32_t CCER;     /* 0x20 */
    __IO uint32_t CNT;      /* 0x24 */
    __IO uint32_t PSC;      /* 0x28 */
    __IO uint32_t ARR;      /* 0x2C */
    __IO uint32_t RCR;      /* 0x30 */
    __IO uint32_t CCR1;     /* 0x34 */
    __IO uint32_t CCR2;     /* 0x38 */
    __IO uint32_t CCR3;     /* 0x3C */
    __IO uint32_t CCR4;     /* 0x40 */
} TIM_TypeDef;

typedef struct {
    __IO uint32_t SR;       /* 0x00 */
    __IO uint32_t DR;       /* 0x04 */
    __IO uint32_t BRR;      /* 0x08 */
    __IO uint32_t CR1;      /* 0x0C */
    __IO uint32_t CR2;      /* 0x10 */
    __IO uint32_t CR3;      /* 0x14 */
    __IO uint32_t GTPR;     /* 0x18 */
} USART_TypeDef;

typedef struct {
    __IO uint32_t CR;       /* 0x00 */
    __IO uint32_t CFGR;     /* 0x04 */
    __IO uint32_t CIR;      /* 0x08 */
    __IO uint32_t APB2RSTR; /* 0x0C */
    __IO uint32_t APB1RSTR; /* 0x10 */
    __IO uint32_t AHBENR;   /* 0x14 */
    __IO uint32_t APB2ENR;  /* 0x18 */
    __IO uint32_t APB1ENR;  /* 0x1C */
    __IO uint32_t BDCR;     /* 0x20 */
    __IO uint32_t CSR;      /* 0x24 */
} RCC_TypeDef;

/*==============================================================================
 * 4. 外设实例
 *============================================================================*/
#define GPIOA               ((GPIO_TypeDef *)0x40010800u)
#define GPIOB               ((GPIO_TypeDef *)0x40010C00u)
#define GPIOC               ((GPIO_TypeDef *)0x40011000u)

#define TIM2                ((TIM_TypeDef *)0x40000000u)
#define TIM3                ((TIM_TypeDef *)0x40000400u)
#define TIM4                ((TIM_TypeDef *)0x40000800u)

#define USART1              ((USART_TypeDef *)0x40013800u)

#define RCC                 ((RCC_TypeDef *)0x40021000u)

/*==============================================================================
 * 5. 常用位与宏
 *============================================================================*/

#define GPIO_Pin_0          (1u << 0)
#define GPIO_Pin_1          (1u << 1)
#define GPIO_Pin_6          (1u << 6)
#define GPIO_Pin_7          (1u << 7)
#define GPIO_Pin_9          (1u << 9)
#define GPIO_Pin_10         (1u << 10)
#define GPIO_Pin_11         (1u << 11)
#define GPIO_Pin_12         (1u << 12)
#define GPIO_Pin_13         (1u << 13)
#define GPIO_Pin_14         (1u << 14)
#define GPIO_Pin_15         (1u << 15)

#define RCC_APB2ENR_AFIOEN      (1u << 0)
#define RCC_APB2ENR_IOPAEN      (1u << 2)
#define RCC_APB2ENR_IOPBEN      (1u << 3)
#define RCC_APB2ENR_IOPCEN      (1u << 4)
#define RCC_APB2ENR_USART1EN    (1u << 14)

#define RCC_APB1ENR_TIM2EN      (1u << 0)
#define RCC_APB1ENR_TIM3EN      (1u << 1)
#define RCC_APB1ENR_TIM4EN      (1u << 2)

#define USART_SR_RXNE           (1u << 5)
#define USART_SR_TXE            (1u << 7)

#define USART_CR1_RE            (1u << 2)
#define USART_CR1_TE            (1u << 3)
#define USART_CR1_RXNEIE        (1u << 5)
#define USART_CR1_TXEIE         (1u << 7)
#define USART_CR1_UE            (1u << 13)

/*==============================================================================
 * 6. 系统时钟（由 system_stm32f10x.c 提供）
 *============================================================================*/
extern uint32_t SystemCoreClock;      /* HCLK，单位 Hz */

void SystemInit(void);
void SystemCoreClockUpdate(void);

#endif /* __STM32F10X_H */
