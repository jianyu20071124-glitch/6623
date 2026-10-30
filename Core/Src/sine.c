/**
  ******************************************************************************
  * @file    sine.c
  * @brief   正弦跟踪发生器（F-08 / T-09）
  * @note    用相位累加器产生正弦目标角：
  *            phase += 2π·f·dt      （每 1 ms 走 0.00314 rad）
  *            target = center + A·sin(phase)
  *          优点：不依赖绝对时间，不会因 tick 溢出而跳变。
  ******************************************************************************
  */
#include "sine.h"
#include <math.h>

#define TWO_PI   6.2831853f
#define DT_S     0.001f          /* 1 ms，与 1 kHz 控制周期一致 */

/* ================= 内部状态 ================= */
static float    s_phase  = 0.0f;   /* 相位 0 ~ 2π */
static float    s_center = 0.0f;   /* 摆动中心（使能时的角度） */
static uint32_t s_t0     = 0;      /* 开始摆动时刻 */
static uint8_t  s_run    = 0;      /* 1 = 正在摆动 */

/* ================= 初始化 ================= */
void Sine_Init(void)
{
    s_phase  = 0.0f;
    s_center = 0.0f;
    s_t0     = 0;
    s_run    = 0;
}

/* ================= 开始摆动 =================
 * center_deg：以哪个角度为中心摆（一般传当前角度，避免阶跃） */
void Sine_Start(float center_deg)
{
    s_center = center_deg;
    s_phase  = 0.0f;             /* 从 sin(0)=0 开始，即从中心出发 */
    s_t0     = HAL_GetTick();
    s_run    = 1;
}

/* ================= 停止摆动 ================= */


/* ================= 每次控制周期更新一次，返回目标角度 =================
 * 相位推进方式：累计调用次数 × 理论单周期 dt，而不是读 HAL_GetTick 差值。
 * 原因：HAL_GetTick() 只有 1 ms 分辨率，而 POS_P_UPDATE 的调用间隔
 *       并不稳定在 1 ms；用 tick 差值会出现"多个周期落在同一 tick → dt=0
 *       被跳过"，导致相位推进比真实时间快（实测输出 0.70 Hz 而非 0.5 Hz）。
 * 现在改为按调用次数推进，频率由 CTRL_DT_S 与中断实际频率共同决定；
 * 若中断是严格 1 kHz，则频率精确等于 SINE_FREQ_HZ。 */
float Sine_Update(void)
{
    if (!s_run)
    {
        return s_center;         /* 停摆时保持在中心 */
    }

    /* ① 相位推进：每个控制周期走 2π·f·Ts */
    s_phase += TWO_PI * SINE_FREQ_HZ * DT_S;

    /* ② 走满一圈归零（避免数值无限增大导致 float 精度下降） */
    if (s_phase > TWO_PI) s_phase -= TWO_PI;

    /* ③ 到时自动停（10 秒），用 tick 判断总时长（这里精度够） */


    /* ④ 目标角度 = 中心 + 30° × sin(相位) */
    return s_center + SINE_AMP_DEG * sinf(s_phase);
}

/* ================= 状态查询 ================= */
uint8_t Sine_IsRunning(void)
{
    return s_run;
}

float Sine_GetCenter(void)
{
    return s_center;
}
