/**
  ******************************************************************************
  * @file    pid.c
  * @brief   通用 PID 控制器实现（纯算法，不依赖电机/角度模块）
  ******************************************************************************
  */
#include "pid.h"
#include <math.h>

/**
  * @brief  初始化 PID 实例
  * @param  d_alpha 微分低通系数：alpha = 1 - e^(-2*pi*fc*dt)
  *                 50 Hz / 1 ms → 0.27
  */
void PID_Init(PID_T *p, float kp, float ki, float kd, float dt,
              float out_max, float i_max, float d_alpha)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->dt = (dt > 0.0f) ? dt : 0.001f;      /* 防除零 */

    p->out_max =  out_max;
    p->out_min = -out_max;
    p->i_max   =  i_max;
    p->i_min   = -i_max;

    p->d_alpha = (d_alpha > 0.0f && d_alpha < 1.0f) ? d_alpha : 0.27f;

    p->integ     = 0.0f;
    p->prev_meas = 0.0f;
    p->d_filt    = 0.0f;
    p->err       = 0.0f;
    p->out       = 0.0f;

    p->primed    = 0;       /* 首拍不计算微分 */
    p->saturated = 0;
    p->err_wrap  = 0;       /* 默认不归一化，角度环由上层置 1 */
    p->i_zone    = 0.0f;    /* 0=不分离；>0 表示只在 |err|<i_zone 时积分 */
}

/**
  * @brief  复位控制器（用当前测量值建立微分基准，避免首拍冲击）
  * @param  meas 当前测量值
  */
void PID_Reset(PID_T *p, float meas)
{
    p->integ     = 0.0f;
    p->prev_meas = meas;    /* 关键：用真实值，不是 0 */
    p->d_filt    = 0.0f;
    p->err       = 0.0f;
    p->out       = 0.0f;
    p->primed    = 1;       /* 已有基准，可以算微分 */
    p->saturated = 0;
}

/**
  * @brief  计算一次输出（每个控制周期调用一次）
  * @param  target 目标值
  * @param  meas   测量值
  * @retval 限幅后的输出
  */
float PID_Calc(PID_T *p, float target, float meas)
{
    if (p->dt <= 0.0f) p->dt = 0.001f;      /* 双保险 */

    /* ① 本拍误差 */
    p->err = target - meas;

    /* 角度环：误差归一化到 ±180°，防过零时误差跳 ±360 */
    if (p->err_wrap)
    {
        while (p->err >  180.5f) p->err -= 360.0f;
        while (p->err < -180.5f) p->err += 360.0f;
    }

    /* ② 微分：作用于测量值，首拍跳过 */
    float d_raw = 0.0f;
    if (p->primed)
    {
        d_raw = (meas - p->prev_meas) / p->dt;
    }
    p->prev_meas = meas;
    p->primed    = 1;

    /* ③ 微分低通（抑制噪声放大） */
    p->d_filt += p->d_alpha * (d_raw - p->d_filt);

    /* ④ 条件积分：上拍输出饱和时不再累加 */
    if ((!p->saturated) &&
        ((p->i_zone <= 0.0f) || (fabsf(p->err) < p->i_zone)))
    {
        p->integ += p->err * p->dt;
        if (p->integ > p->i_max) p->integ = p->i_max;
        if (p->integ < p->i_min) p->integ = p->i_min;
    }

    /* ⑤ PID 求和 */
    float out = p->kp * p->err
              + p->ki * p->integ
              - p->kd * p->d_filt;

    /* ⑥ 记录本拍饱和状态（供下一拍条件积分） */
    p->saturated = (out > p->out_max) || (out < p->out_min);

    /* ⑦ 输出限幅 */
    if (out > p->out_max) out = p->out_max;
    if (out < p->out_min) out = p->out_min;
    p->out = out;

    return out;
}
