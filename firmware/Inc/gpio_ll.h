/**
 ******************************************************************************
 * @file    gpio_ll.h
 * @brief   GPIO 引脚配置的寄存器级封装（只依赖 CMSIS，不依赖标准外设库）
 *
 * 为什么要自己写这一层：
 *   标准外设库(SPL)有一百多个文件，新手光配环境就要卡半天；
 *   本工程只用到 GPIO/TIM/USART 三种外设，寄存器直接写反而更透明，
 *   编译也只需要 CMSIS 头文件 + 启动文件，PlatformIO 一条命令搞定。
 *
 * STM32 每个引脚用 CRL/CRH 里的 4 个位描述：
 *   [MODE 2bit][CNF 2bit]
 *   MODE: 00=输入  01=输出10MHz  10=输出2MHz  11=输出50MHz
 *   CNF(输入): 00=模拟 01=浮空 10=上下拉 11=保留
 *   CNF(输出): 00=推挽 01=开漏 10=复用推挽 11=复用开漏
 ******************************************************************************
 */
#ifndef __GPIO_LL_H
#define __GPIO_LL_H

#include "stm32f10x.h"

/* 常用配置值（MODE|CNF 拼好的 4 位） */
#define GPIO_CONF_IN_FLOAT     0x4u   /* 浮空输入 */
#define GPIO_CONF_IN_PULL      0x8u   /* 上下拉输入（ODR 决定上/下拉） */
#define GPIO_CONF_OUT_PP_2M    0x2u   /* 通用推挽输出 2MHz */
#define GPIO_CONF_OUT_PP_50M   0x3u   /* 通用推挽输出 50MHz */
#define GPIO_CONF_AF_PP_50M    0xBu   /* 复用推挽输出 50MHz */

/** 配置单个引脚（端口时钟必须已由调用方打开） */
static inline void gpio_config_pin(GPIO_TypeDef *port, uint8_t pin, uint8_t conf)
{
    if (pin < 8u) {
        uint32_t shift = (uint32_t)pin * 4u;
        port->CRL = (port->CRL & ~(0xFu << shift)) | ((uint32_t)conf << shift);
    } else {
        uint32_t shift = ((uint32_t)pin - 8u) * 4u;
        port->CRH = (port->CRH & ~(0xFu << shift)) | ((uint32_t)conf << shift);
    }
}

/** 一次配置同一端口的一组引脚（pin_mask 为 GPIO_Pin_x 的或） */
static inline void gpio_config_mask(GPIO_TypeDef *port, uint32_t pin_mask,
                                    uint8_t conf)
{
    uint8_t i;
    for (i = 0u; i < 16u; i++) {
        if ((pin_mask & (1u << i)) != 0u) {
            gpio_config_pin(port, i, conf);
        }
    }
}

/** 上下拉模式时，ODR=1 选上拉、ODR=0 选下拉 */
static inline void gpio_set_pullup(GPIO_TypeDef *port, uint32_t pin_mask)
{
    port->BSRR = pin_mask;
}

#endif /* __GPIO_LL_H */
