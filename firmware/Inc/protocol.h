/**
 ******************************************************************************
 * @file    protocol.h
 * @brief   上位机 <-> 底盘 的 ASCII 行协议
 *
 * 为什么用 ASCII 而不是二进制：
 *   调试阶段能直接在串口助手里肉眼读，出问题一眼定位；
 *   代价是带宽，115200 下 50 Hz 上报 + 20 Hz 指令完全够用。
 *
 * 指令（一行一条，\r\n 结尾，回车即生效）：
 *   V <v> <w>          设置线速度/角速度，例：V 0.30 0.00
 *   S                  立即停车
 *   Z                  编码器与里程计清零
 *   P <kp> <ki> <kd>   在线改速度环参数，例：P 1800 90 0
 *   R                  恢复默认参数
 *   ?                  查询当前参数
 *
 * 上报（50 Hz）：
 *   T,<ms>,<vL>,<vR>,<tgtL>,<tgtR>,<tickL>,<tickR>,<dutyL>,<dutyR>,<lost>
 ******************************************************************************
 */
#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>

/** 解析一行（不含结束符）；返回 0 = 未识别，1 = 已处理 */
uint8_t protocol_handle_line(const char *line);

/** 把当前底盘快照打包成一行上报文本，写入 out（容量 cap），返回字节数 */
uint16_t protocol_build_telemetry(char *out, uint16_t cap);

/** SysTick(1 kHz) 里调用：只做计数，禁止任何字符串操作 */
void protocol_tick_flag(void);

/** 主循环里调用：收字节、解析、按 50 Hz 节流上报 */
void protocol_poll(void);

/** 确保发送中断处于开启状态（换 DMA 的接入点） */
void protocol_flush(void);

#endif /* __PROTOCOL_H */
