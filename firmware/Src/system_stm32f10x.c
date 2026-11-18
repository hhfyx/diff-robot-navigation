/**
 ******************************************************************************
 * @file    system_stm32f10x.c
 * @brief   系统时钟初始化：HSE 8 MHz -> PLL x9 -> 72 MHz
 *
 * 蓝丸(Blue Pill)板载 8 MHz 晶振，标准配置就是 72 MHz。
 * 必须同时保证：
 *   AHB  = 72 MHz (HCLK)
 *   APB2 = 72 MHz (PCLK2, USART1/TIM1 用)
 *   APB1 = 36 MHz (PCLK1, TIM2/3/4 用；定时器时钟自动 x2 = 72 MHz)
 *
 * 若你的板子晶振是 12 MHz，改 HSE_VALUE 后重新编译即可。
 ******************************************************************************
 */
#include "stm32f10x.h"

#define HSE_VALUE          8000000u
#define HSI_VALUE          8000000u

/* RCC_CR 位 */
#define RCC_CR_HSION       (1u << 0)
#define RCC_CR_HSIRDY      (1u << 1)
#define RCC_CR_HSEON       (1u << 16)
#define RCC_CR_HSERDY      (1u << 17)
#define RCC_CR_PLLON       (1u << 24)
#define RCC_CR_PLLRDY      (1u << 25)

/* RCC_CFGR 位 */
#define RCC_CFGR_SW_PLL    (2u << 0)
#define RCC_CFGR_SWS_PLL   (2u << 2)
#define RCC_CFGR_HPRE_DIV1 (0u << 4)
#define RCC_CFGR_PPRE1_DIV2 (4u << 8)     /* APB1 = HCLK/2 = 36 MHz */
#define RCC_CFGR_PPRE2_DIV1 (0u << 11)    /* APB2 = HCLK   = 72 MHz */
#define RCC_CFGR_PLLSRC_HSE (1u << 16)
#define RCC_CFGR_PLLXTPRE_DIV1 (0u << 17)
#define RCC_CFGR_PLLMULL9  (7u << 18)     /* x9 */
#define RCC_CFGR_SWS_MASK  (3u << 2)
#define RCC_CFGR_PLLSRC    (1u << 16)

/* Flash 等待周期：72 MHz 需要 2 个等待周期 */
#define FLASH_ACR          (*(__IO uint32_t *)0x40022000u)
#define FLASH_ACR_LATENCY_2 (2u << 0)
#define FLASH_ACR_PRFTBE   (1u << 4)

uint32_t SystemCoreClock = 72000000u;

/**
 * 配置 72 MHz 时钟树。
 * 失败时自动回退到 HSI 8 MHz，保证程序至少还能跑（串口会变慢到 115200/9）。
 */
void SystemInit(void)
{
    uint32_t timeout;

    /* 1. 先确保 HSI 开着（复位后默认就是开的） */
    RCC->CR |= RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0u) { }

    /* 2. 打开 HSE 并等待起振 */
    RCC->CR |= RCC_CR_HSEON;
    timeout = 0x100000u;
    while (((RCC->CR & RCC_CR_HSERDY) == 0u) && (timeout > 0u)) {
        timeout--;
    }

    /* HSE 起振失败：退回 8 MHz 内部时钟，程序仍可运行（只是变慢） */
    if (timeout == 0u) {
        RCC->CFGR = RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE2_DIV1;
        SystemCoreClock = HSI_VALUE;
        FLASH_ACR = FLASH_ACR_PRFTBE;
        return;
    }

    /* 3. Flash 等待周期与预取 */
    FLASH_ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

    /* 4. 配置 PLL：HSE(不分频) x9 = 72 MHz
     *    同时配好 AHB/APB 分频，一步到位避免中间态超频 */
    RCC->CFGR = RCC_CFGR_HPRE_DIV1 |
                RCC_CFGR_PPRE1_DIV2 |
                RCC_CFGR_PPRE2_DIV1 |
                RCC_CFGR_PLLSRC_HSE |
                RCC_CFGR_PLLXTPRE_DIV1 |
                RCC_CFGR_PLLMULL9;

    /* 5. 使能 PLL 并等待锁定 */
    RCC->CR |= RCC_CR_PLLON;
    timeout = 0x100000u;
    while (((RCC->CR & RCC_CR_PLLRDY) == 0u) && (timeout > 0u)) {
        timeout--;
    }
    if (timeout == 0u) {
        /* PLL 起不来，同样退回 HSI */
        RCC->CR &= ~RCC_CR_PLLON;
        RCC->CFGR = RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE2_DIV1;
        SystemCoreClock = HSI_VALUE;
        return;
    }

    /* 6. 切换系统时钟源到 PLL */
    RCC->CFGR = (RCC->CFGR & ~0x3u) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_PLL) { }

    SystemCoreClock = 72000000u;
}

void SystemCoreClockUpdate(void)
{
    /* 本工程固定 72 MHz（或回退 8 MHz），无需动态推算 */
}
