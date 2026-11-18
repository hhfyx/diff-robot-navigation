/**
 ******************************************************************************
 * @file    usart.h
 * @brief   USART1 调试串口：中断收 + 环形缓冲，发送非阻塞
 *
 * 环形缓冲区是"中断里只搬字节、主循环里做解析"的关键，
 * 中断服务函数里绝不做字符串解析或长耗时运算。
 ******************************************************************************
 */
#ifndef __USART_H
#define __USART_H

#include <stdint.h>

void usart1_init(uint32_t baud);

/* ---- 接收侧 ---- */
/** 取一个收到的字节；返回 1 = 取到，0 = 空 */
uint8_t usart1_getc(uint8_t *byte);
/** 接收溢出计数（环形缓冲满导致丢字节） */
uint32_t usart1_rx_overflow(void);

/* ---- 发送侧 ---- */
/** 非阻塞发送：缓冲区有空位就塞，满了直接丢弃新数据（宁可丢日志不阻塞控制） */
void usart1_puts(const char *s);
void usart1_write(const uint8_t *data, uint16_t len);
/** 发送环形缓冲剩余待发字节数 */
uint16_t usart1_tx_pending(void);

#endif /* __USART_H */
