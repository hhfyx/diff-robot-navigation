/**
 ******************************************************************************
 * @file    main.c
 * @brief   蓝丸两轮差速底盘 —— 主程序
 *
 * 运行结构（对应原文 3.4 那节的取舍）：
 *
 *   SysTick 1 kHz   优先级 1
 *      ├─ chassis_tick()       编码器采样 + 斜坡 + PID + PWM
 *      └─ protocol_tick_flag() 只做 tick_cnt++
 *
 *   USART1_IRQ      优先级 2：只搬字节进环形缓冲
 *
 *   while(1)        优先级最低：解析指令、格式化遥测、发送、闪灯
 *
 * 控制节拍里绝不出现字符串操作或浮点格式化，所以串口再忙
 * 也拉不长 1 ms 周期 —— 这正是原文"小车一顿一顿"问题的根因。
 ******************************************************************************
 */
#include "stm32f10x.h"
#include "board.h"
#include "chassis.h"
#include "protocol.h"
#include "usart.h"
#include "gpio_ll.h"

/* SysTick 中断：控制节拍 */
void SysTick_Handler(void)
{
    chassis_tick();          /* 采样 -> 斜坡 -> PID -> 写 CCR */
    protocol_tick_flag();    /* 只累加计数 */
}

static void led_init(void)
{
    RCC->APB2ENR |= (1u << 4);                 /* GPIOC 时钟 */
    gpio_config_pin(LED_PORT, 13u, GPIO_CONF_OUT_PP_2M);
    LED_PORT->BSRR = LED_PIN;                  /* 高电平 = 灭 */
}

static void led_toggle(void)
{
    if ((LED_PORT->ODR & LED_PIN) != 0u) {
        LED_PORT->BRR = LED_PIN;               /* 低电平点亮 */
    } else {
        LED_PORT->BSRR = LED_PIN;
    }
}

/* 启动提示用的忙等，不进控制回路 */
static void delay_rough(volatile uint32_t n)
{
    while (n-- > 0u) {
        __NOP();
    }
}

int main(void)
{
    uint32_t led_cnt = 0u;
    uint8_t  i;

    /* SystemInit() 由启动文件在 main 之前调用，这里不用再调。
     * 它把 HSE 8 MHz x9 配成 72 MHz（bluepill 默认）。 */

    led_init();
    chassis_init();                 /* 编码器 + 电机 + 双 PID */
    usart1_init(DBG_BAUDRATE);

    usart1_puts("\r\n=== DiffBot-BLUEPILL v1.0 ===\r\n");
    usart1_puts("cmd: V <v> <w> | S stop | Z zero | P kp ki kd | R reset | ?\r\n");

    /* 启动自检三次快闪 */
    for (i = 0u; i < 3u; i++) {
        LED_PORT->BRR  = LED_PIN;
        delay_rough(400000u);
        LED_PORT->BSRR = LED_PIN;
        delay_rough(400000u);
    }

    /* 开控制节拍：1 kHz */
    (void)SysTick_Config(SystemCoreClock / CTRL_TICK_HZ);
    NVIC_SetPriority(SysTick_IRQn, 1u);   /* 必须高于 USART1 的 2 */

    while (1) {
        protocol_poll();           /* 收字节 -> 解析 -> 50 Hz 上报 */
        protocol_flush();

        /* 心跳灯约 1 Hz（与 1 kHz 控制节拍无关） */
        led_cnt++;
        if (led_cnt >= 400000u) {
            led_cnt = 0u;
            led_toggle();
        }

        /* 断链后常亮：一眼区分"上位机掉线"和"车本身坏了" */
        if (chassis_is_link_lost() != 0u) {
            LED_PORT->BRR = LED_PIN;
        }
    }
}

/* ---------------------------------------------------------------------------
 * 异常处理
 * -------------------------------------------------------------------------*/
void HardFault_Handler(void)
{
    __disable_irq();
    while (1) {
        __NOP();
    }
}

void NMI_Handler(void)        { __disable_irq(); while (1) { __NOP(); } }
void MemManage_Handler(void)  { __disable_irq(); while (1) { __NOP(); } }
void BusFault_Handler(void)   { __disable_irq(); while (1) { __NOP(); } }
void UsageFault_Handler(void) { __disable_irq(); while (1) { __NOP(); } }
void SVC_Handler(void)        { }
void DebugMon_Handler(void)   { }
void PendSV_Handler(void)     { }
