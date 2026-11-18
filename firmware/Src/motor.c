/**
 ******************************************************************************
 * @file    motor.c
 * @brief   TB6612 鍙岃矾椹卞姩 + TIM4 鍙岄€氶亾 PWM 鈥斺€?绾?CMSIS 瀵勫瓨鍣ㄥ疄鐜? *
 * 寮曡剼锛? *   PB6  TIM4_CH1 -> PWMA (宸?
 *   PB7  TIM4_CH2 -> PWMB (鍙?
 *   PB11 -> STBY
 *   PB12/PB13 -> AIN1/AIN2 (宸?
 *   PB14/PB15 -> BIN1/BIN2 (鍙?
 *
 * 鐢垫簮锛堝師鏂囬偅鏉¤娉暀璁紝鍔″繀鐓у仛锛夛細
 *   閫昏緫 3.3 V 浠庢澘瀛?3V3 鍙栵紱鐢垫満 VM 鍗曠嫭绮楃嚎鍒扮數姹狅紱
 *   涓よ€呭叡鍦帮紝浣嗗湴绾垮垎寮€璧板埌鍚屼竴涓帴鍦扮偣锛岀粷涓嶅厑璁哥數鏈虹數娴佷粠鏉垮瓙鐨? *   3V3 鍦扮嚎涓婃祦杩?鈥斺€?鍚﹀垯鍚姩鐬棿鐨勫帇闄嶄細鐩存帴鎶婂崟鐗囨満鎷夊浣嶃€? ******************************************************************************
 */
#include "motor.h"
#include "board.h"
#include "gpio_ll.h"

/* TIM4 PWM 閰嶇疆浣?*/
#define TIM_CR1_CEN      (1u << 0)
#define TIM_CR1_ARPE     (1u << 7)
#define TIM_EGR_UG       (1u << 0)

#define CCMR1_OC1M_PWM1  (6u << 4)     /* OC1M = 110 */
#define CCMR1_OC1PE      (1u << 3)
#define CCMR1_OC2M_PWM1  (6u << 12)
#define CCMR1_OC2PE      (1u << 11)
#define CCER_CC1E        (1u << 0)
#define CCER_CC2E        (1u << 4)

static void gpio_init(void)
{
    /* GPIOB 鏃堕挓 */
    RCC->APB2ENR |= (1u << 3);

    /* PB6/PB7锛氬鐢ㄦ帹鎸?50 MHz锛?xB锛?*/
    gpio_config_pin(TB_PORT, 6u, GPIO_CONF_AF_PP_50M);
    gpio_config_pin(TB_PORT, 7u, GPIO_CONF_AF_PP_50M);

    /* PB11~PB15锛氶€氱敤鎺ㄦ尳杈撳嚭 50 MHz锛?x3锛?*/
    gpio_config_pin(TB_PORT, 11u, GPIO_CONF_OUT_PP_50M);
    gpio_config_pin(TB_PORT, 12u, GPIO_CONF_OUT_PP_50M);
    gpio_config_pin(TB_PORT, 13u, GPIO_CONF_OUT_PP_50M);
    gpio_config_pin(TB_PORT, 14u, GPIO_CONF_OUT_PP_50M);
    gpio_config_pin(TB_PORT, 15u, GPIO_CONF_OUT_PP_50M);

    /* 鍒濆鍏ㄩ儴鎷変綆锛歋TBY 浣庯紙寰呮満锛夛紝鏂瑰悜鑴氫綆锛堟儼琛岋級 */
    TB_PORT->BRR = TB_STBY_PIN | TB_AIN1_PIN | TB_AIN2_PIN |
                   TB_BIN1_PIN | TB_BIN2_PIN;
}

static void pwm_init(void)
{
    /* TIM4 鎸傚湪 APB1銆侫PB1 涓?36 MHz锛屼絾瀹氭椂鍣ㄦ椂閽熻嚜鍔?x2 = 72 MHz */
    RCC->APB1ENR |= (1u << 2);

    TIM4->PSC = 0u;
    TIM4->ARR = PWM_ARR;

    /* CH1/CH2 閮介厤鎴?PWM1 妯″紡锛屽苟浣胯兘棰勮杞斤紙鏀?CCR 鏃跺湪鏇存柊浜嬩欢鐢熸晥锛?     * 閬垮厤涓€涓懆鏈熷唴鍑虹幇鍗婃尝姣涘埡锛?*/
    TIM4->CCMR1 = CCMR1_OC1M_PWM1 | CCMR1_OC1PE |
                  CCMR1_OC2M_PWM1 | CCMR1_OC2PE;

    TIM4->CCER  = CCER_CC1E | CCER_CC2E;

    TIM4->CCR1  = 0u;
    TIM4->CCR2  = 0u;

    TIM4->EGR  |= TIM_EGR_UG;
    TIM4->CR1  |= TIM_CR1_ARPE | TIM_CR1_CEN;
}

void motor_init(void)
{
    gpio_init();
    pwm_init();

    /* 鎷夐珮 STBY 浣胯兘 TB6612 */
    TB_PORT->BSRR = TB_STBY_PIN;
}

/** 闄愬箙鍒?卤ARR銆傛敞鎰忥細杩欓噷銆愪笉鍋氥€戞鍖鸿ˉ鍋裤€? *
 *  鏃╁厛鐨勭増鏈妸浠讳綍闈為浂鍗犵┖姣旀姮鍒?MOTOR_DEADBAND(180)锛? *  缁撴灉 PID 杈撳嚭钀藉湪 (0, 180] 鏃舵墽琛屽櫒鎭掑畾杈撳嚭 180 鈥斺€?杈撳嚭涓嶈繛缁紝
 *  绉垎椤瑰け鍘绘剰涔夛紝瀹炴祴浼氬舰鎴愭瀬闄愮幆锛屽皬鐩爣閫熷害鏍规湰鏀朵笉浣忋€? *
 *  姝ｇ‘鍋氭硶锛氳鎵ц鍣ㄤ繚鎸佺嚎鎬э紝姝诲尯浜ょ粰 PID 鐨勭Н鍒嗛」鑷繁鍘婚《銆? *  绉垎瓒冲澶ф椂杈撳嚭鑷劧鐖繃 180锛岀數鏈哄氨杞捣鏉ヤ簡锛? *  鑰屼笖绋虫€佹椂绉垎姝ｅソ绛変簬姝诲尯鐢靛帇锛岃繖鏄閲忓紡 PID 鐨勫ぉ鐒朵紭鍔裤€?*/
static uint16_t clamp_duty(int16_t duty_abs)
{
    if ((uint16_t)duty_abs > PWM_ARR) {
        return (uint16_t)PWM_ARR;
    }
    return (uint16_t)duty_abs;
}

static int16_t clamp_pwm(int16_t d)
{
    if (d >  (int16_t)PWM_ARR) return  (int16_t)PWM_ARR;
    if (d < -(int16_t)PWM_ARR) return -(int16_t)PWM_ARR;
    return d;
}

void motor_set_left(int16_t duty)
{
    int16_t d = clamp_pwm(duty);

    if (d > 0) {
        TB_PORT->BSRR = TB_AIN1_PIN;            /* AIN1=1 */
        TB_PORT->BRR  = TB_AIN2_PIN;            /* AIN2=0 */
    } else if (d < 0) {
        TB_PORT->BRR  = TB_AIN1_PIN;            /* AIN1=0 */
        TB_PORT->BSRR = TB_AIN2_PIN;            /* AIN2=1 */
        d = (int16_t)(-d);
    } else {
        /* 鎯拌锛氫袱鑴氶兘浣庯紝鐢垫満鑷敱婊戣锛堟瘮鍒硅溅鍐插嚮灏忥級 */
        TB_PORT->BRR = TB_AIN1_PIN | TB_AIN2_PIN;
    }

    PWM_L_CCR = clamp_duty(d);
}

void motor_set_right(int16_t duty)
{
    int16_t d = clamp_pwm(duty);

    if (d > 0) {
        TB_PORT->BSRR = TB_BIN1_PIN;
        TB_PORT->BRR  = TB_BIN2_PIN;
    } else if (d < 0) {
        TB_PORT->BRR  = TB_BIN1_PIN;
        TB_PORT->BSRR = TB_BIN2_PIN;
        d = (int16_t)(-d);
    } else {
        TB_PORT->BRR = TB_BIN1_PIN | TB_BIN2_PIN;
    }

    PWM_R_CCR = clamp_duty(d);
}

void motor_brake_all(void)
{
    /* 涓ょ浉鐭矾鍒硅溅锛欼N1=IN2=1 */
    TB_PORT->BSRR = TB_AIN1_PIN | TB_AIN2_PIN | TB_BIN1_PIN | TB_BIN2_PIN;
    PWM_L_CCR = PWM_ARR;
    PWM_R_CCR = PWM_ARR;
}

void motor_coast_all(void)
{
    PWM_L_CCR = 0u;
    PWM_R_CCR = 0u;
    TB_PORT->BRR = TB_AIN1_PIN | TB_AIN2_PIN | TB_BIN1_PIN | TB_BIN2_PIN;
    /* STBY 淇濇寔楂樼數骞筹細鍙嶅鎷変綆鍐嶆媺楂樹細璁?TB6612 鍐呴儴閲嶆柊涓婄數銆?     * 浜х敓涓€娆＄數娴佸啿鍑伙紝闀挎湡涓嬫潵瀹规槗鐑ч┍鍔?*/
}
