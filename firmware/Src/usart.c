/**
 ******************************************************************************
 * @file    usart.c
 * @brief   USART1 中断收发 + 双环形缓冲区 —— 纯 CMSIS 寄存器实现
 *
 * 设计原则：
 *   中断里只做"搬字节"，解析/格式化/PID 全在主循环和 SysTick 里；
 *   发送绝不阻塞，缓冲区满就丢新数据（日志可以丢，控制不能停）。
 ******************************************************************************
 */
#include "usart.h"
#include "board.h"
#include "gpio_ll.h"

/* USART1 寄存器位 */
#define USART_SR_RXNE      (1u << 5)
#define USART_SR_TXE       (1u << 7)
#define USART_CR1_RE       (1u << 2)
#define USART_CR1_TE       (1u << 3)
#define USART_CR1_RXNEIE   (1u << 5)
#define USART_CR1_TXEIE    (1u << 7)
#define USART_CR1_UE       (1u << 13)

/* NVIC */
#define NVIC_ISER1        (*(volatile uint32_t *)0xE000E104u)
#define NVIC_IPR_BASE     (0xE000E400u)

#define RX_BUF_SIZE   256u
#define TX_BUF_SIZE   512u

static volatile uint8_t  rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint32_t rx_overflow;

static volatile uint8_t  tx_buf[TX_BUF_SIZE];
static volatile uint16_t tx_head;
static volatile uint16_t tx_tail;

void usart1_init(uint32_t baud)
{
    uint32_t pclk2;
    uint32_t brr;

    /* GPIOA 时钟 + USART1 时钟（APB2） */
    RCC->APB2ENR |= (1u << 2) | (1u << 14);

    /* PA9 = TX 复用推挽 50MHz（0xB），PA10 = RX 浮空输入（0x4） */
    gpio_config_pin(USART1_TX_PORT, 9u,  GPIO_CONF_AF_PP_50M);
    gpio_config_pin(USART1_RX_PORT, 10u, GPIO_CONF_IN_FLOAT);

    /* PCLK2 = 72 MHz（SystemCoreClock 就是 sysclk，USART1 挂在 APB2） */
    pclk2 = SystemCoreClock;
    brr   = (pclk2 + baud / 2u) / baud;
    USART1->BRR = (uint16_t)brr;

    rx_head = rx_tail = 0u;
    tx_head = tx_tail = 0u;
    rx_overflow = 0u;

    /* 8N1 + 收/发使能 + 打开发送缓冲区空中断（队列空时会在 ISR 里关掉） */
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

    /* USART1_IRQn = 37 => ISER1 的 bit (37-32)=5
     * 抢占优先级写成 (2 << 4)，即抢占 2、子优先级 0。
     * 必须低于 SysTick 的 1 —— 原文那个"串口抢占定时器导致小车一顿一顿"
     * 的坑，本质上就是这里配反了。 */
    *(volatile uint8_t *)(NVIC_IPR_BASE + 37u) = (uint8_t)(2u << 4);
    NVIC_ISER1 = (1u << (37u - 32u));
}

/* ---------------------------- 接收 ---------------------------- */

uint8_t usart1_getc(uint8_t *byte)
{
    if (rx_tail == rx_head) {
        return 0u;
    }
    *byte = rx_buf[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1u) % RX_BUF_SIZE);
    return 1u;
}

uint32_t usart1_rx_overflow(void)
{
    return rx_overflow;
}

/* ---------------------------- 发送 ---------------------------- */

void usart1_write(const uint8_t *data, uint16_t len)
{
    uint16_t i;

    for (i = 0u; i < len; i++) {
        uint16_t next = (uint16_t)((tx_head + 1u) % TX_BUF_SIZE);
        if (next == tx_tail) {
            break;              /* 满：丢弃剩余数据，绝不阻塞 */
        }
        tx_buf[tx_head] = data[i];
        tx_head = next;
    }

    USART1->CR1 |= USART_CR1_TXEIE;
}

void usart1_puts(const char *s)
{
    while (*s != '\0') {
        usart1_write((const uint8_t *)s, 1u);
        s++;
    }
}

uint16_t usart1_tx_pending(void)
{
    return (uint16_t)((tx_head + TX_BUF_SIZE - tx_tail) % TX_BUF_SIZE);
}

/* ------------------------- 中断服务函数 ------------------------- */

void USART1_IRQHandler(void)
{
    uint32_t sr = USART1->SR;

    /* 收 */
    if ((sr & USART_SR_RXNE) != 0u) {
        uint8_t  b    = (uint8_t)(USART1->DR & 0xFFu);   /* 读 DR 自动清标志 */
        uint16_t next = (uint16_t)((rx_head + 1u) % RX_BUF_SIZE);

        if (next != rx_tail) {
            rx_buf[rx_head] = b;
            rx_head = next;
        } else {
            rx_overflow++;
        }
    }

    /* 发 */
    if ((sr & USART_SR_TXE) != 0u) {
        if (tx_tail != tx_head) {
            USART1->DR = tx_buf[tx_tail];
            tx_tail = (uint16_t)((tx_tail + 1u) % TX_BUF_SIZE);
        } else {
            USART1->CR1 &= ~USART_CR1_TXEIE;   /* 队列空，关中断省 CPU */
        }
    }
}
