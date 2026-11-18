/**
 ******************************************************************************
 * @file    protocol.c
 * @brief   ASCII 行协议解析 + 遥测上报
 *
 * 职责划分（对应原文 3.4 的"主循环里做非阻塞"）：
 *   - SysTick 中断（1 kHz）里只调 protocol_tick_flag()，就是一句 tick_cnt++；
 *   - 所有字符串解析、格式化、发送都放在主循环的 protocol_poll() 里。
 *   这样无论串口数据多密，都不会把 1 ms 控制节拍拉长。
 *
 * 注意：这里刻意不用 printf("%f")。
 *   Keil MDK 默认 microlib 不支持浮点格式化，勾选 "Use MicroLIB" 后
 *   %f 会输出空或乱码；改用定点整数手工拼接，任何编译选项下都稳。
 ******************************************************************************
 */
#include "protocol.h"
#include "board.h"
#include "chassis.h"
#include "usart.h"

/* chassis.c 提供的在线调参接口 */
void chassis_set_gains(float kp, float ki, float kd);
void chassis_get_gains(float *kp, float *ki, float *kd);
void chassis_reset_gains(void);
void chassis_zero_odom(void);

#define TELEMETRY_HZ      50u
#define TELEMETRY_DIV     (CTRL_TICK_HZ / TELEMETRY_HZ)   /* = 20 */

static volatile uint32_t tick_cnt;      /* SysTick 里自增 */
static uint32_t          tele_last;     /* 主循环侧记录上次上报的 tick */

static char    line_buf[96];
static uint8_t line_len;

/* ------------------------------------------------------------------
 * 极简字符串 -> float（不用 atof，避免拖入完整 libc）
 * 支持 "0.30" "-1.5" "12"
 * ------------------------------------------------------------------ */
static float str_to_float(const char *s)
{
    float   sign = 1.0f;
    float   val  = 0.0f;
    float   frac = 0.1f;
    uint8_t dot  = 0;

    if (*s == '-') { sign = -1.0f; s++; }
    else if (*s == '+') { s++; }

    while (*s != '\0' && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') {
        if (*s == '.') {
            dot = 1;
        } else if (*s >= '0' && *s <= '9') {
            if (!dot) {
                val = val * 10.0f + (float)(*s - '0');
            } else {
                val += (float)(*s - '0') * frac;
                frac *= 0.1f;
            }
        } else {
            break;
        }
        s++;
    }
    return sign * val;
}

/* 定点输出：按 3 位小数写入 dst，返回写入长度（不含结尾 '\0'） */
static uint8_t fmt_fixed3(char *dst, float x)
{
    char    tmp[16];
    uint8_t n = 0;
    uint8_t k;
    int32_t v;
    int32_t ip;
    int32_t fp;

    if (x < 0.0f) { *dst++ = '-'; n++; x = -x; }

    v = (int32_t)(x * 1000.0f + 0.5f);     /* 四舍五入到 0.001 */
    if (v > 9999999) { v = 9999999; }

    ip = v / 1000;
    fp = v % 1000;

    if (ip == 0) {
        tmp[0] = '0'; k = 1;
    } else {
        k = 0;
        while (ip > 0 && k < 10) {
            tmp[k++] = (char)('0' + (ip % 10));
            ip /= 10;
        }
    }
    while (k > 0) { *dst++ = tmp[--k]; n++; }

    *dst++ = '.';                                n++;
    *dst++ = (char)('0' + (fp / 100) % 10);      n++;
    *dst++ = (char)('0' + (fp / 10) % 10);       n++;
    *dst++ = (char)('0' + (fp % 10));            n++;

    *dst = '\0';
    return n;
}

/* 有符号整数输出，返回写入长度 */
static uint8_t fmt_int(char *dst, int32_t v)
{
    char    tmp[12];
    uint8_t k = 0;
    uint8_t n = 0;

    if (v < 0) { *dst++ = '-'; n++; v = -v; }

    if (v == 0) {
        *dst++ = '0';
        *dst   = '\0';
        return (uint8_t)(n + 1u);
    }
    while (v > 0 && k < 11) {
        tmp[k++] = (char)('0' + (v % 10));
        v /= 10;
    }
    while (k > 0) { *dst++ = tmp[--k]; n++; }

    *dst = '\0';
    return n;
}

/* ------------------------------------------------------------------
 * 遥测打包
 * ------------------------------------------------------------------ */
uint16_t protocol_build_telemetry(char *out, uint16_t cap)
{
    ChassisState_t s = chassis_snapshot();
    char    *p    = out;
    uint16_t used = 0;

    if (cap < 96u) {
        return 0;
    }

#define PUT_S(str) do { \
        const char *q_ = (str); \
        while (*q_ != '\0') { *p++ = *q_++; used++; } \
    } while (0)
#define PUT_F(x)   do { p += fmt_fixed3(p, (x)); used = (uint16_t)(p - out); } while (0)
#define PUT_I(x)   do { p += fmt_int(p, (int32_t)(x)); used = (uint16_t)(p - out); } while (0)

    PUT_S("T,");
    PUT_I((int32_t)(tick_cnt & 0x7FFFFFFF));  PUT_S(",");
    PUT_F(s.v_left);                          PUT_S(",");
    PUT_F(s.v_right);                         PUT_S(",");
    PUT_F(s.v_target);                        PUT_S(",");
    PUT_F(s.w_target);                        PUT_S(",");
    PUT_I(s.ticks_left);                      PUT_S(",");
    PUT_I(s.ticks_right);                     PUT_S(",");
    PUT_I(s.duty_left);                       PUT_S(",");
    PUT_I(s.duty_right);                      PUT_S(",");
    PUT_I((int32_t)s.link_lost);
    PUT_S("\r\n");

#undef PUT_S
#undef PUT_F
#undef PUT_I

    return used;
}

/* ------------------------------------------------------------------
 * 指令解析
 * ------------------------------------------------------------------ */
static const char *skip_spaces(const char *s)
{
    while (*s == ' ' || *s == '\t') { s++; }
    return s;
}

static const char *skip_token(const char *s)
{
    while (*s != '\0' && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') {
        s++;
    }
    return s;
}

uint8_t protocol_handle_line(const char *line)
{
    const char *p;

    if (line == 0 || *line == '\0') {
        return 0;
    }

    p = skip_spaces(line);

    switch (*p) {
    case 'V': case 'v': {
        const char *a = skip_spaces(p + 1);
        float v = str_to_float(a);
        a = skip_spaces(skip_token(a));
        float w = str_to_float(a);

        chassis_set_cmd(v, w);
        return 1;
    }

    case 'S': case 's':
        chassis_stop();
        usart1_puts("OK,STOP\r\n");
        return 1;

    case 'Z': case 'z':
        chassis_zero_odom();
        usart1_puts("OK,ZERO\r\n");
        return 1;

    case 'P': case 'p': {
        const char *a = skip_spaces(p + 1);
        float kp = str_to_float(a);
        a = skip_spaces(skip_token(a));
        float ki = str_to_float(a);
        a = skip_spaces(skip_token(a));
        float kd = str_to_float(a);

        chassis_set_gains(kp, ki, kd);
        usart1_puts("OK,PID\r\n");
        return 1;
    }

    case 'R': case 'r':
        chassis_reset_gains();
        usart1_puts("OK,RESET\r\n");
        return 1;

    case '?': {
        float kp, ki, kd;
        char  buf[64];
        char *q = buf;
        const char *tag = "PID,";

        while (*tag != '\0') { *q++ = *tag++; }

        chassis_get_gains(&kp, &ki, &kd);
        q += fmt_fixed3(q, kp); *q++ = ',';
        q += fmt_fixed3(q, ki); *q++ = ',';
        q += fmt_fixed3(q, kd);
        *q++ = '\r'; *q++ = '\n'; *q = '\0';

        usart1_puts(buf);
        return 1;
    }

    default:
        return 0;
    }
}

/* ------------------------------------------------------------------
 * SysTick 侧：只做计数，不做任何字符串操作
 * ------------------------------------------------------------------ */
void protocol_tick_flag(void)
{
    tick_cnt++;
}

/* ------------------------------------------------------------------
 * 主循环侧：收字节 -> 解析 -> 50 Hz 上报
 * ------------------------------------------------------------------ */
void protocol_poll(void)
{
    uint8_t b;

    while (usart1_getc(&b)) {
        if (b == '\r' || b == '\n') {
            if (line_len > 0u) {
                line_buf[line_len] = '\0';
                protocol_handle_line(line_buf);
                line_len = 0u;
            }
        } else if (line_len < (uint8_t)(ARRAY_LEN(line_buf) - 1u)) {
            line_buf[line_len++] = (char)b;
        } else {
            line_len = 0u;     /* 超长行丢弃，防止协议错位 */
        }
    }

    /* 50 Hz 上报：基于 SysTick 计数节流，主循环跑多快都不影响上报周期 */
    if ((uint32_t)(tick_cnt - tele_last) >= TELEMETRY_DIV) {
        char     buf[96];
        uint16_t n;

        tele_last = tick_cnt;
        n = protocol_build_telemetry(buf, (uint16_t)sizeof(buf));
        if (n > 0u) {
            usart1_write((const uint8_t *)buf, n);
        }
    }
}

void protocol_flush(void)
{
    /* 发送由 TXE 中断自动驱动，这里只保证中断是开着的。
     * 将来换 DMA 只改这一处。 */
    if (usart1_tx_pending() > 0u) {
        USART1->CR1 |= USART_CR1_TXEIE;
    }
}
