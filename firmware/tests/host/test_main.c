/**
 ******************************************************************************
 * @file    test_main.c
 * @brief   固件核心算法的主机侧验证（不需要板子，直接在 PC 上跑）
 *
 * 验证内容：
 *   1. PID 增量式算法能收敛到目标值，且不超调、不积分饱和
 *   2. 编码器脉冲 -> 线速度的换算系数与手算一致
 *   3. 斜坡发生器的加速/减速速率符合设定
 *   4. 串口 ASCII 协议能正确解析 V/S/P/Z/? 指令
 *   5. 遥测打包格式能被上位机逐字段解析
 ******************************************************************************
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "pid.h"
#include "board.h"

/* ---- 被测试的协议层接口 ---- */
uint8_t  protocol_handle_line(const char *line);
uint16_t protocol_build_telemetry(char *out, uint16_t cap);

/* protocol.c 依赖的外部符号：这里给出测试桩 */
static float g_stub_cmd_v, g_stub_cmd_w;
static float g_stub_kp, g_stub_ki, g_stub_kd;
static int   g_stub_stop_cnt;
static int   g_stub_zero_cnt;
static int   g_stub_reset_cnt;
static char  g_stub_tx[512];
static int   g_stub_tx_len;

void chassis_set_cmd(float v, float w) { g_stub_cmd_v = v; g_stub_cmd_w = w; }
void chassis_set_gains(float kp, float ki, float kd)
{
    g_stub_kp = kp; g_stub_ki = ki; g_stub_kd = kd;
}
void chassis_get_gains(float *kp, float *ki, float *kd)
{
    *kp = g_stub_kp; *ki = g_stub_ki; *kd = g_stub_kd;
}
void chassis_reset_gains(void) { g_stub_reset_cnt++; }
void chassis_zero_odom(void)   { g_stub_zero_cnt++; }
void chassis_stop(void)        { g_stub_stop_cnt++; }

void usart1_puts(const char *s)
{
    int n = (int)strlen(s);
    if (g_stub_tx_len + n < (int)sizeof(g_stub_tx)) {
        memcpy(g_stub_tx + g_stub_tx_len, s, (size_t)n);
        g_stub_tx_len += n;
    }
}
void usart1_write(const uint8_t *d, uint16_t len)
{
    if (g_stub_tx_len + (int)len < (int)sizeof(g_stub_tx)) {
        memcpy(g_stub_tx + g_stub_tx_len, d, len);
        g_stub_tx_len += (int)len;
    }
}
uint16_t usart1_tx_pending(void) { return 0; }
uint8_t  usart1_getc(uint8_t *byte) { (void)byte; return 0; }
uint32_t usart1_rx_overflow(void) { return 0; }

/* chassis_snapshot 也要桩掉（protocol_build_telemetry 用） */
#include "chassis.h"
ChassisState_t chassis_snapshot(void)
{
    ChassisState_t s;
    memset(&s, 0, sizeof(s));
    s.v_left = 0.25f;  s.v_right = 0.26f;
    s.v_target = 0.30f; s.w_target = 0.31f;
    s.ticks_left = 1234; s.ticks_right = -567;
    s.duty_left = 800;  s.duty_right = -300;
    s.link_lost = 0;
    return s;
}

/* protocol_tick_flag 里用到的 tick_cnt 在 protocol.c 内部，这里不管 */
void protocol_tick_flag(void);

/*============================================================================
 * 测试框架
 *==========================================================================*/
static int g_pass, g_fail;

static void check(int cond, const char *name, const char *detail)
{
    if (cond) {
        g_pass++;
        printf("  [PASS] %s\n", name);
    } else {
        g_fail++;
        printf("  [FAIL] %s  %s\n", name, detail ? detail : "");
    }
}

/*============================================================================
 * 1. PID 测试
 *==========================================================================*/
static void test_pid(void)
{
    PID_t p;
    float fb = 0.0f;
    float target = 0.30f;          /* 目标速度 m/s */
    int   i;
    float max_over = 0.0f;
    float final_err;

    printf("\n[1] PID 增量式收敛测试\n");

    /* 被控对象模型（与实体一致）：
     *   满占空比 PWM_ARR=3599 对应空载 ~0.80 m/s => K = 2.22e-4 (m/s)/CCR
     *   一阶惯性 tau = 100 ms
     *   真实死区 180：占空比不足时电机完全不动，超出的部分才转 */
    {
        const float K   = 0.80f / (float)PWM_ARR;
        const float tau = 0.1f;

        PID_Init(&p, PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT,
                 (float)PWM_ARR);

        for (i = 0; i < 3000; i++) {           /* 模拟 3 秒 @1kHz */
            float out  = PID_UpdateIncremental(&p, target, fb, 0.001f);
            float duty = out;

            if (duty >  (float)PWM_ARR) duty =  (float)PWM_ARR;
            if (duty < -(float)PWM_ARR) duty = -(float)PWM_ARR;

            if (fabsf(duty) < (float)MOTOR_DEADBAND) {
                duty = 0.0f;
            } else {
                duty -= (duty > 0.0f) ? (float)MOTOR_DEADBAND
                                      : -(float)MOTOR_DEADBAND;
            }

            fb += (K * duty - fb) * (0.001f / tau);

            if (fb > max_over) { max_over = fb; }
        }

        final_err = fabsf(target - fb);

        printf("      模型: 满占空比=0.80 m/s, tau=100 ms, 死区=%d\n",
               (int)MOTOR_DEADBAND);
        printf("      参数: Kp=%.1f Ki=%.1f Kd=%.1f\n",
               (double)PID_KP_DEFAULT, (double)PID_KI_DEFAULT,
               (double)PID_KD_DEFAULT);
        printf("      最终速度 %.4f m/s, 目标 %.4f m/s, 稳态误差 %.5f m/s\n",
               (double)fb, (double)target, (double)final_err);
        printf("      最大超调 %.4f m/s (%.1f%%)\n",
               (double)max_over, (double)((max_over - target) / target * 100.0f));

        check(final_err < 0.005f, "3 秒内收敛到目标 (误差 < 5 mm/s)", NULL);
        check(max_over < target * 1.25f, "超调 < 25%", NULL);
    }

    /* 限幅测试 */
    PID_Init(&p, 5000.0f, 100.0f, 0.0f, 1000.0f);
    for (i = 0; i < 100; i++) {
        (void)PID_UpdateIncremental(&p, 10.0f, 0.0f, 0.001f);
    }
    check(p.output <= 1000.0f + 0.001f, "输出严格限幅", NULL);

    /* 反向测试 */
    PID_Init(&p, 3000.0f, 300.0f, 0.0f, (float)PWM_ARR);
    for (i = 0; i < 100; i++) {
        (void)PID_UpdateIncremental(&p, -0.2f, 0.0f, 0.001f);
    }
    check(p.output < -100.0f, "负目标产生反向输出", NULL);
}

/*============================================================================
 * 2. 单位换算测试
 *==========================================================================*/
static void test_units(void)
{
    float r, cpr, mpc;
    float v_expected, v_computed;

    printf("\n[2] 编码器脉冲 -> 线速度换算测试\n");

    r   = WHEEL_RADIUS_M;
    cpr = COUNTS_PER_OUT_REV;
    mpc = METERS_PER_COUNT;

    /* 手算：输出轴转 1 圈 = 1600 脉冲 = 2πR 米 */
    check(fabsf(mpc - (2.0f * 3.14159265f * r / cpr)) < 1e-9f,
          "METERS_PER_COUNT 与手算一致", NULL);

    /* 场景：10 ms 窗口内数到 100 个脉冲，轮径 62.5 mm
     * 期望线速度 = 100 * 0.0002454 / 0.01 ≈ 2.454 m/s？不对 —— 实际是
     * 100 脉冲 /1600 脉冲每圈 = 0.0625 圈 => 0.0625*2π*0.0625 = 0.02454 m
     * 除以 0.01 s = 2.454 m/s。这里用 40 脉冲更接近真实：0.98 m/s */
    {
        int   counts = 40;
        float window_s = (float)SPEED_WINDOW_MS * 0.001f;

        v_computed = (float)counts * mpc / window_s;

        /* 手算：40/1600=0.025 圈；0.025*2π*0.0625 = 0.009817 m；/0.01s */
        v_expected = 0.025f * 2.0f * 3.14159265f * 0.0625f / 0.01f;

        check(fabsf(v_computed - v_expected) < 1e-5f,
              "10ms 窗口 40 脉冲 = 0.98 m/s", NULL);
        printf("      计算值 %.4f m/s, 期望 %.4f m/s\n",
               (double)v_computed, (double)v_expected);
    }

    /* 分辨率检查：1 个脉冲对应的速度跳变 */
    {
        float win_s = (float)SPEED_WINDOW_MS * 0.001f;
        float res = mpc / win_s;
        printf("      单脉冲速度分辨率 %.4f m/s\n", (double)res);
        check(res < 0.03f, "分辨率优于 0.03 m/s（10ms 窗口）", NULL);
    }
}

/*============================================================================
 * 3. 斜坡发生器测试
 *==========================================================================*/
static void test_ramp(void)
{
    float cur = 0.0f;
    float dst = 1.0f;
    float up_step = MAX_ACCEL_MPS2 * CTRL_DT_S;
    float dn_step = MAX_DECEL_MPS2 * CTRL_DT_S;
    int   ticks = 0;

    printf("\n[3] 斜坡发生器测试\n");

    /* 0 -> 1 m/s，加速度 1 m/s^2，1 kHz 节拍 => 需要 1000 拍 */
    while (cur < dst - 1e-6f && ticks < 20000) {
        float d = dst - cur;
        if (d > up_step) d = up_step;
        cur += d;
        ticks++;
    }

    printf("      0->1 m/s 用了 %d 拍 (= %.2f s)\n", ticks, ticks * 0.001f);
    check(ticks >= 999 && ticks <= 1001,
          "加速时间符合 a=1 m/s^2 的预期 (1.000 s)", NULL);

    /* 减速：2 m/s^2，应该 500 拍 */
    cur = 1.0f;
    ticks = 0;
    while (cur > 0.0f && ticks < 20000) {
        float d = 0.0f - cur;
        if (d < -dn_step) d = -dn_step;
        cur += d;
        ticks++;
    }
    printf("      1->0 m/s 用了 %d 拍 (= %.2f s)\n", ticks, ticks * 0.001f);
    check(ticks >= 499 && ticks <= 501,
          "减速时间符合 a=2 m/s^2 的预期 (0.500 s)", NULL);
}

/*============================================================================
 * 4/5. 协议测试
 *==========================================================================*/
static void test_protocol(void)
{
    char buf[128];
    uint16_t n;

    printf("\n[4] 串口协议解析测试\n");

    g_stub_cmd_v = g_stub_cmd_w = 0.0f;
    check(protocol_handle_line("V 0.30 0.00") == 1, "V 指令被识别", NULL);
    check(fabsf(g_stub_cmd_v - 0.30f) < 1e-3f, "线速度解析正确", NULL);
    check(fabsf(g_stub_cmd_w - 0.00f) < 1e-6f, "角速度解析正确", NULL);

    check(protocol_handle_line("V -0.15 0.25") == 1, "V 指令支持负值", NULL);
    check(fabsf(g_stub_cmd_v + 0.15f) < 1e-3f, "负线速度解析正确", NULL);
    check(fabsf(g_stub_cmd_w - 0.25f) < 1e-3f, "正角速度解析正确", NULL);

    check(protocol_handle_line("S") == 1 && g_stub_stop_cnt == 1,
          "S 指令触发停车", NULL);

    check(protocol_handle_line("Z") == 1 && g_stub_zero_cnt == 1,
          "Z 指令触发清零", NULL);

    check(protocol_handle_line("P 2000 100 5") == 1, "P 指令被识别", NULL);
    check(fabsf(g_stub_kp - 2000.0f) < 1e-3f &&
          fabsf(g_stub_ki - 100.0f) < 1e-3f &&
          fabsf(g_stub_kd - 5.0f) < 1e-3f,
          "PID 三参数在线修改成功", NULL);

    check(protocol_handle_line("R") == 1 && g_stub_reset_cnt == 1,
          "R 指令恢复默认参数", NULL);

    /* 未知指令不应崩溃 */
    check(protocol_handle_line("X 1 2 3") == 0, "未知指令被安全忽略", NULL);
    check(protocol_handle_line("") == 0, "空行被安全忽略", NULL);

    printf("\n[5] 遥测打包测试\n");

    g_stub_tx_len = 0;
    n = protocol_build_telemetry(buf, (uint16_t)sizeof(buf));

    check(n > 0, "遥测打包成功", NULL);
    check(strncmp(buf, "T,", 2) == 0, "帧头为 T,", NULL);
    check(strstr(buf, "\r\n") != NULL, "帧尾为 \\r\\n", NULL);

    /* 逐字段核对：应为 11 个字段（T + 10 个数据） */
    {
        int commas = 0;
        const char *q = buf;
        while (*q) { if (*q == ',') commas++; q++; }
        printf("      帧内容: %s", buf);
        printf("      字段数(=逗号数+1): %d\n", commas + 1);
        check(commas == 10, "共 11 个字段", NULL);
    }

    check(strstr(buf, "0.250") != NULL, "左轮速度格式化为 3 位小数", NULL);
    check(strstr(buf, "-567") != NULL, "负里程正确输出", NULL);
}

/*============================================================================
 * 入口
 *==========================================================================*/
int main(void)
{
    printf("==================================================\n");
    printf(" DiffBot 固件核心算法主机侧验证\n");
    printf("==================================================\n");
    printf("参数: R=%.4f m  CPR=%.0f  轮距=%.3f m  窗口=%u ms\n",
           (double)WHEEL_RADIUS_M, (double)COUNTS_PER_OUT_REV,
           (double)WHEEL_SEPARATION_M, (unsigned)SPEED_WINDOW_MS);

    test_pid();
    test_units();
    test_ramp();
    test_protocol();

    printf("\n==================================================\n");
    printf(" 结果: %d 通过, %d 失败\n", g_pass, g_fail);
    printf("==================================================\n");

    return (g_fail == 0) ? 0 : 1;
}
