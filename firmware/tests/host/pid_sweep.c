/**
 * PID 参数扫描：为默认增益找一组实测可用值
 *
 * 被控对象（物理上准确的模型）：
 *   - 真实死区：占空比 < 180 电机不动（负载+静摩擦）
 *   - 一阶惯性：满占空比 3599 对应 0.80 m/s，tau = 100 ms
 *
 * 关键设计决策：死区不靠执行器"补偿"，靠 PID 积分项自己顶过去。
 * 这样不会像"把占空比抬到 180"那样产生不连续的输出跳变和极限环。
 */
#include <stdio.h>
#include <math.h>
#include "pid.h"
#include "board.h"

static void sim(float kp, float ki, float kd, float target,
                float *overshoot, float *settle_ms, float *err, float *fb_out)
{
    PID_t p;
    float fb = 0.0f;
    int   i;
    int   settled = -1;
    const float K    = 0.80f / (float)PWM_ARR;
    const float tau  = 0.1f;
    float mx = 0.0f;

    PID_Init(&p, kp, ki, kd, (float)PWM_ARR);

    for (i = 0; i < 6000; i++) {
        float out  = PID_UpdateIncremental(&p, target, fb, 0.001f);
        float duty = out;

        if (duty >  (float)PWM_ARR) duty =  (float)PWM_ARR;
        if (duty < -(float)PWM_ARR) duty = -(float)PWM_ARR;

        /* 真实死区：电压不足以克服静摩擦，电机完全不动 */
        if (fabsf(duty) < (float)MOTOR_DEADBAND) {
            duty = 0.0f;
        } else {
            /* 超过死区后，扣除死区后的部分才转化为转速 */
            duty = duty - (duty > 0.0f ? (float)MOTOR_DEADBAND
                                       : -(float)MOTOR_DEADBAND);
        }

        fb += (K * duty - fb) * (0.001f / tau);
        if (fb > mx) mx = fb;

        if (settled < 0 && fabsf(target - fb) < 0.02f * target) {
            settled = i;
        } else if (settled >= 0 && fabsf(target - fb) > 0.05f * target) {
            settled = -1;
        }
    }

    *overshoot = (mx - target) / target * 100.0f;
    *settle_ms = (settled < 0) ? -1.0f : (float)settled;
    *err       = fabsf(target - fb);
    *fb_out    = fb;
}

int main(void)
{
    float targets[] = { 0.10f, 0.30f, 0.60f };
    float kps[] = { 500, 800, 1200, 1800, 2500, 3500 };
    float kis[] = { 2000, 5000, 10000, 20000, 40000 };
    int k, j, t;
    int shown = 0;

    printf("target   Kp     Ki    os%%  settle_ms     err      fb\n");
    for (t = 0; t < 3; t++) {
        for (k = 0; k < 6; k++) {
            for (j = 0; j < 5; j++) {
                float os, st, er, fb;
                sim(kps[k], kis[j], 0.0f, targets[t], &os, &st, &er, &fb);
                if (er < 0.003f && os < 25.0f && st >= 0 && st < 1200) {
                    printf("%.2f  %6.0f %6.0f  %5.1f  %8.0f  %.5f  %.4f\n",
                           (double)targets[t], (double)kps[k], (double)kis[j],
                           (double)os, (double)st, (double)er, (double)fb);
                    shown++;
                }
            }
        }
    }
    printf("\n命中 %d 组（稳态<3mm/s, 超调<25%%, 调节<1.2s）\n", shown);
    return 0;
}
