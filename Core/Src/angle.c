/**
  ******************************************************************************
  * @file    angle.c
  * @brief   角度解算与速度估计（F-04）—— 由你自己实现
  * @note    速度估计：1 ms 差分 + 一阶低通（IIR, α=0.02），无滑动平均
  ******************************************************************************
  */
#include "angle.h"
#include "motor_6623.h"

/* ============ 输出变量（定义在这里，供 telemetry 读取）============ */
volatile float   g_angle_deg  = 0.0f;   /* 单圈角度 0~360° */
volatile float   g_angle_cont = 0.0f;   /* 连续角度（含多圈，原始） */
volatile float   g_omega_rpm  = 0.0f;   /* 角速度 rpm（一阶低通后，给速度环） */
volatile float   g_omega_iir_rpm = 0.0f;/* 角速度 rpm（一阶低通，控制用）*/
volatile float   g_omega_raw_rpm = 0.0f;/* 角速度 rpm（未滤波，供延迟测量） */
volatile int32_t g_turns      = 0;      /* 多圈计数 */

/* ================= 滤波系数 =================
 * 速度反馈：一阶低通 α=0.02 → fc≈3.2 Hz，延迟约 50 ms
 *   必须滤：1 ms 差分把 0.0439° 的量化误差放大成 7.32 rpm/count，
 *   不滤时静止噪声 ±15 rpm、150 rpm 直接失控到 254（实测）。
 *   位置反馈：不滤波，直接用编码器原始值（本身就是 0.0439° 分辨率，够干净）。 */
#define IIR_ALPHA   0.02f    /* 速度反馈：fc≈3.2 Hz，延迟约 50 ms */

static int16_t  s_last_raw = 0;    /* 角度 */
static uint32_t s_last_ms  = 0;    /* 时刻 */

void Angle_Update(void)
{uint32_t now = HAL_GetTick();
  uint32_t dt  = now - s_last_ms;

  if (dt == 0u) return;
  s_last_ms = now;
  int16_t raw = (int16_t)g_angle_raw;
  int32_t delta = (int32_t)raw - (int32_t)s_last_raw;
  /* raw 增大为正方向：跨零时 raw 由 8191 跳到 0（delta 为大负数）→ 圈数 +1 */
  if (delta >  4096) { delta -= 8192; g_turns--; }   /* raw 由 0 跳到 8191 → 反转过零 */
  if (delta < -4096) { delta += 8192; g_turns++; }   /* raw 由 8191 跳到 0 → 正转过零 */
  s_last_raw = raw;
  g_angle_deg  = (float)raw * (360.0f / 8192.0f);
  g_angle_cont = (float)g_turns * 360.0f + g_angle_deg;

  float omega_dps = ((float)delta * (360.0f / 8192.0f)) * 1000.0f / (float)dt;

  /* ---- 速度：一阶低通后给速度环 ----
     必须滤：1 ms 差分把编码器量化误差放大 167 倍（7.32 rpm/count），
     不滤时静止噪声 ±15 rpm、150 rpm 会失控到 254。
     α=0.02 → fc≈3.2 Hz，延迟约 50 ms（实测超调最低点）*/
  float rpm_now = omega_dps / 6.0f;                    /* °/s → rpm */
  g_omega_iir_rpm += IIR_ALPHA * (rpm_now - g_omega_iir_rpm);
  g_omega_rpm = g_omega_iir_rpm;                       /* 作为控制用转速 */
}
