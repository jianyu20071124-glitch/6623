/**
  ******************************************************************************
  * @file    cascade.c
  * @brief   位置-速度串级控制（F-06 / F-07 主力方案）
  * @note    外环 PID：角度误差 → 转速目标（rpm），带限速
  *          内环调用 speed.c 的 Speed_Calc()：转速误差 → 电流（count）
  *          中间加摩擦前馈，解决位置环最后几度补不上的问题
  ******************************************************************************
  */
#include "cascade.h"
#include "speed.h"
#include "angle.h"
#include <math.h>

/* ================= 内部状态 ================= */
static PID_T s_pos;                /* 位置环（外环）*/

/* 速度环当前的目标转速（对外暴露，供串口显示） */
volatile float g_rpm_target = 0.0f;

/* ================= 初始化 ================= */
void CASC_Init(void)
{
    PID_Init(&s_pos,
             CASC_POS_KP, CASC_POS_KI, CASC_POS_KD,
             0.001f,                 /* 1 ms */
             CASC_POS_OUT_MAX,       /* 输出限幅 = 转速限幅 */
             CASC_POS_INT_MAX,
             0.27f);                 /* 微分低通 50 Hz */

    s_pos.err_wrap = 1;              /* 角度误差归一化（180° 阶跃不翻车）*/
    s_pos.i_zone   = 5.0f;           /* 积分分离：仅 |err|<5° 才积分 */
}

/* ================= 算一次电流 ================= */
float CASC_Calc(float target_deg, float meas_deg)
{
    /* ① 外环：位置误差 → 转速目标 */
    float rpm_cmd = PID_Calc(&s_pos, target_deg, meas_deg);

    /* ② 限速（不是限电流）—— 靠它把转速压在安全范围内 */
    if (rpm_cmd >  CASC_POS_OUT_MAX) rpm_cmd =  CASC_POS_OUT_MAX;
    if (rpm_cmd < -CASC_POS_OUT_MAX) rpm_cmd = -CASC_POS_OUT_MAX;

    /* 对外暴露"速度环当前的目标转速"，供串口显示 */
    g_rpm_target = rpm_cmd;

    /* ③ 内环：转速目标 → 电流 */
    float iq = Speed_Calc(rpm_cmd);

    /* ④ 摩擦前馈：帮内环越过低速死区 */
    if (rpm_cmd >  CASC_FF_TH) iq += CASC_FF_POS;
    if (rpm_cmd < -CASC_FF_TH) iq -= CASC_FF_NEG;

    /* ⑤ 限幅 */
    if (iq >  CASC_IQ_MAX) iq =  CASC_IQ_MAX;
    if (iq < -CASC_IQ_MAX) iq = -CASC_IQ_MAX;

    return iq;
}

/* ================= 取外环 PID ================= */
PID_T *CASC_Get_PosPID(void)
{
    return &s_pos;
}
