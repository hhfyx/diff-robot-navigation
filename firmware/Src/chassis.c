/**
 ******************************************************************************
 * @file    chassis.c
 * @brief   底盘核心：测速 + 斜坡 + 双轮独立 PID
 *
 * 单位统一约定（全部 SI，写进注释避免后面排查扯皮）：
 *   位置 m，速度 m/s，角速度 rad/s，时间 s，占空比 = 整数 CCR(0~PWM_ARR)
 ******************************************************************************
 */
#include "chassis.h"
#include "board.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"

/* ---- 左右轮各自一套 PID，严禁共用变量 ---- */
static PID_t pid_l;
static PID_t pid_r;

/* ---- 测速滑窗 ----
 * 环里有 SPEED_HIST_LEN 个 1 ms 采样，最老与最新之间正好跨
 * (SPEED_HIST_LEN - 1) = SPEED_WINDOW_MS 个 1 ms 间隔，
 * 所以下面按 SPEED_WINDOW_MS ms 换算，单位严格对应。 */
#define SPEED_HIST_LEN (SPEED_WINDOW_MS + 1u)

static uint16_t hist_l[SPEED_HIST_LEN];
static uint16_t hist_r[SPEED_HIST_LEN];
static uint8_t  hist_idx;
static uint8_t  hist_fill;

/* ---- 累计里程（int32，可负） ---- */
static int32_t  ticks_l;
static int32_t  ticks_r;
static uint16_t last_cnt_l;
static uint16_t last_cnt_r;

/* ---- 斜坡后的目标 [m/s] ---- */
static float ramp_tgt_l;
static float ramp_tgt_r;

/* ---- 上位机原始指令 ---- */
static float cmd_v;
static float cmd_w;

/* ---- 链路超时 ---- */
static uint32_t link_idle_ms;
static uint8_t  link_lost;

static ChassisState_t st;

/* ---------------------------------------------------------------------------
 * 工具函数
 * -------------------------------------------------------------------------*/

/** 16 位 CNT 差分 -> 有符号 32 位增量（自动处理 65535/0 回绕） */
static int32_t cnt_delta(uint16_t now, uint16_t prev)
{
    return (int32_t)(int16_t)(uint16_t)(now - prev);
}

/** 斜坡发生器：让 cur 以每拍 up_step / dn_step 的速率逼近 dst */
static float ramp_toward(float cur, float dst, float up_step, float dn_step)
{
    float d = dst - cur;

    if (d > 0.0f) {
        if (d > up_step) { d = up_step; }
    } else if (d < 0.0f) {
        if (d < -dn_step) { d = -dn_step; }
    }
    return cur + d;
}

void chassis_init(void)
{
    encoder_init();
    motor_init();

    PID_Init(&pid_l, PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT,
             PID_OUT_LIMIT);
    PID_Init(&pid_r, PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT,
             PID_OUT_LIMIT);

    hist_idx   = 0u;
    hist_fill  = 0u;
    ticks_l    = 0;
    ticks_r    = 0;
    last_cnt_l = encoder_raw(0u);
    last_cnt_r = encoder_raw(1u);

    ramp_tgt_l = 0.0f;
    ramp_tgt_r = 0.0f;
    cmd_v      = 0.0f;
    cmd_w      = 0.0f;

    link_idle_ms = 0u;
    link_lost    = 0u;

    st.v_left = 0.0f;  st.v_right = 0.0f;
    st.v_target = 0.0f; st.w_target = 0.0f;
    st.v = 0.0f;       st.w = 0.0f;
    st.ticks_left = 0; st.ticks_right = 0;
    st.duty_left = 0;  st.duty_right = 0;
    st.link_lost = 0u;
}

void chassis_set_cmd(float v, float w)
{
    /* 底盘级限幅：保证 (v ± w·B/2) 都不超单轮上限。
     * 否则转弯时外侧轮被 PID 限幅切掉，实际轨迹会明显内切。 */
    float half = WHEEL_SEPARATION_M * 0.5f;
    float outer = v + (w > 0.0f ? w * half : -w * half);

    if ((w != 0.0f) && (outer > MAX_LINEAR_SPEED_MPS)) {
        float scale = MAX_LINEAR_SPEED_MPS / outer;
        v *= scale;
        w *= scale;
    }

    if (v >  MAX_LINEAR_SPEED_MPS) { v =  MAX_LINEAR_SPEED_MPS; }
    if (v < -MAX_LINEAR_SPEED_MPS) { v = -MAX_LINEAR_SPEED_MPS; }

    cmd_v = v;
    cmd_w = w;

    link_idle_ms = 0u;      /* 收到指令即刷新链路心跳 */
    link_lost    = 0u;
}

void chassis_stop(void)
{
    cmd_v      = 0.0f;
    cmd_w      = 0.0f;
    ramp_tgt_l = 0.0f;
    ramp_tgt_r = 0.0f;

    PID_Reset(&pid_l);
    PID_Reset(&pid_r);

    motor_coast_all();

    st.duty_left  = 0;
    st.duty_right = 0;
}

void chassis_tick(void)
{
    uint16_t cnt_l;
    uint16_t cnt_r;
    int32_t  d_l;
    int32_t  d_r;
    float    step_up;
    float    step_dn;
    float    tgt_l;
    float    tgt_r;
    float    out_l;
    float    out_r;

    /* ---------------- 1. 采样编码器 ---------------- */
    cnt_l = encoder_raw(0u);
    cnt_r = encoder_raw(1u);

    d_l = cnt_delta(cnt_l, last_cnt_l);
    d_r = cnt_delta(cnt_r, last_cnt_r);
    last_cnt_l = cnt_l;
    last_cnt_r = cnt_r;

    ticks_l += d_l;
    ticks_r += d_r;

    hist_l[hist_idx] = cnt_l;
    hist_r[hist_idx] = cnt_r;
    hist_idx++;
    if (hist_idx >= SPEED_HIST_LEN) {
        hist_idx  = 0u;
        hist_fill = 1u;
    }

    /* ---------------- 2. 滑窗测速 ----------------
     * 用窗口首尾两次 CNT 之差 / 窗口时长 = 固定时间间隔测速，
     * 但每 1 ms 都能刷新一次结果（滑动），响应不滞后。 */
    if (hist_fill != 0u) {
        float span_l = (float)cnt_delta(cnt_l, hist_l[hist_idx]);
        float span_r = (float)cnt_delta(cnt_r, hist_r[hist_idx]);
        float win_s  = (float)SPEED_WINDOW_MS * 0.001f;   /* 窗口时长 s */

        float v_l = span_l * METERS_PER_COUNT / win_s;    /* m/s */
        float v_r = span_r * METERS_PER_COUNT / win_s;

        /* 窗口本身已平滑，这里只做轻微去毛刺（一阶低通 α=0.5） */
        st.v_left  = 0.5f * st.v_left  + 0.5f * v_l;
        st.v_right = 0.5f * st.v_right + 0.5f * v_r;
    }

    /* ---------------- 3. 差速运动学反解 + 斜坡 ---------------- */
    {
        float half   = WHEEL_SEPARATION_M * 0.5f;
        float want_l = cmd_v - cmd_w * half;
        float want_r = cmd_v + cmd_w * half;

        step_up = MAX_ACCEL_MPS2 * CTRL_DT_S;
        step_dn = MAX_DECEL_MPS2 * CTRL_DT_S;

        /* 目标先过斜坡再进 PID：启动无电流冲击，
         * 也避免阶跃被 Kd 放大成噪声尖峰 */
        ramp_tgt_l = ramp_toward(ramp_tgt_l, want_l, step_up, step_dn);
        ramp_tgt_r = ramp_toward(ramp_tgt_r, want_r, step_up, step_dn);

        tgt_l = ramp_tgt_l;
        tgt_r = ramp_tgt_r;
    }

    /* ---------------- 4. 双轮独立 PID ---------------- */
    out_l = PID_UpdateIncremental(&pid_l, tgt_l, st.v_left,  CTRL_DT_S);
    out_r = PID_UpdateIncremental(&pid_r, tgt_r, st.v_right, CTRL_DT_S);

    motor_set_left((int16_t)out_l);
    motor_set_right((int16_t)out_r);

    /* ---------------- 5. 链路超时保护 ----------------
     * 上位机 20 Hz 下发时，500 ms 无消息即自动停车，
     * 防止遥控端掉线后小车继续满速冲出去。 */
    if (link_idle_ms < 0xFFFFFFFFu) {
        link_idle_ms++;
    }
    if ((link_lost == 0u) && (link_idle_ms > LINK_TIMEOUT_MS)) {
        link_lost = 1u;
        chassis_stop();
    }

    /* ---------------- 6. 更新快照 ---------------- */
    st.v_target = tgt_l;
    st.w_target = tgt_r;
    st.v  = 0.5f * (st.v_left + st.v_right);
    st.w  = (st.v_right - st.v_left) / WHEEL_SEPARATION_M;
    st.ticks_left  = ticks_l;
    st.ticks_right = ticks_r;
    st.duty_left   = (int16_t)out_l;
    st.duty_right  = (int16_t)out_r;
    st.link_lost   = link_lost;
}

ChassisState_t chassis_snapshot(void)
{
    return st;
}

uint8_t chassis_is_link_lost(void)
{
    return link_lost;
}

/* ---- 供协议层在线调参 ---- */
void chassis_set_gains(float kp, float ki, float kd)
{
    pid_l.Kp = kp; pid_l.Ki = ki; pid_l.Kd = kd;
    pid_r.Kp = kp; pid_r.Ki = ki; pid_r.Kd = kd;
}

void chassis_get_gains(float *kp, float *ki, float *kd)
{
    if (kp != 0) { *kp = pid_l.Kp; }
    if (ki != 0) { *ki = pid_l.Ki; }
    if (kd != 0) { *kd = pid_l.Kd; }
}

void chassis_reset_gains(void)
{
    chassis_set_gains(PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT);
}

void chassis_zero_odom(void)
{
    encoder_zero();
    last_cnt_l = 0u;
    last_cnt_r = 0u;
    ticks_l    = 0;
    ticks_r    = 0;
}
