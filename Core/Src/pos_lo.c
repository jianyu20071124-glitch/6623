/**
  ******************************************************************************
  * @file    pos_lo.c
  * @brief   位置单环控制器（F-06）：位置误差 -> 电流指令
  * @note    单一 PID 从角度误差直接算电流。
  *          额外加"死区补偿"：PID 输出落在摩擦死区内时，强制抬到死区以上，
  *          否则误差还剩几度、电流却不够推动电机，稳态永远补不上。
  ******************************************************************************
  */
#include "pos_lo.h"
#include <math.h>

/* ================= 内部状态 ================= */
static PID_T s_pid;

/* ================= 初始化 ================= */
void PL_Init(void)
{
    PID_Init(&s_pid,
             PL_KP, PL_KI, PL_KD,
             0.001f,              /* 1 ms */
             PL_OUT_MAX,          /* 输出限幅 = 电流限幅 ±5000 */
             PL_INT_MAX,
             0.27f);              /* 微分低通 50 Hz（Kd=0 时无效）*/

    s_pid.err_wrap = 1;           /* 角度环：误差归一化到 ±180°，走最短路径 */
    s_pid.i_zone   = PL_I_ZONE;   /* 积分分离：仅 |err|<5° 时积分 */
}

/* ================= 算一次电流 ================= */
float PL_Calc(float target_deg, float meas_deg)
{
    /* ① 位置误差 -> 电流（PID 内部含 ±PL_OUT_MAX 限幅）*/
    float u = PID_Calc(&s_pid, target_deg, meas_deg);

    /* ② 死区补偿：误差还没消掉、但输出不够跨过摩擦死区时，
          强制把输出抬到 ±PL_DZ_COMP，保证电机一定动得起来。
          误差小于 PL_DZ_ERR 时不补偿，避免在目标点来回推（极限环）。 */
    if (fabsf(s_pid.err) > PL_DZ_ERR)
    {
        if ((u > -PL_DZ_COMP) && (u < PL_DZ_COMP))
        {
            u = (s_pid.err > 0.0f) ? PL_DZ_COMP : -PL_DZ_COMP;
        }
    }

    /* ④ 二次限幅（防御性，防参数被误改）*/
    if (u >  PL_OUT_MAX) u =  PL_OUT_MAX;
    if (u < -PL_OUT_MAX) u = -PL_OUT_MAX;

    return u;
}

/* ================= 取 PID 实例 ================= */
PID_T *PL_Get_PID(void)
{
    return &s_pid;
}
