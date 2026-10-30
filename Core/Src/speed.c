/**
  ******************************************************************************
  * @file    speed.c
  * @brief   速度环：转速目标 -> 电流指令（F-07 / T-08）
  * @note    对象辨识结果 w/u = 0.1091/(0.009s+1)，死区 22 count
  ******************************************************************************
  */
#include "speed.h"
#include "motor_6623.h"
#include "angle.h"

/* ================= 实例与目标 ================= */
PID_T           pid_spd;
volatile float  g_spd_target = 0.0f;

/* ================= 参数（调参改这里） =================
 * Kp = 5   : 开环增益 0.1091*5 = 0.55，闭环 tau~5.7 ms，带宽~28 Hz
 *            速度反馈延迟 4.5 ms 在 28 Hz 处滞后约 68 度，相位裕度约 62 度
 *            Kp > 10 时带宽 > 40 Hz，延迟滞后 > 100 度，会振荡
 * Ki = 190 : Ti = Kp/Ki ~ 26 ms ~ 3*tau，消除纯 P 时 65% 的稳态误差
 * 限幅 3000: 300 rpm / 0.1091 = 2750 count，+-500 只能到 54 rpm
 */
#define SPD_KP         20.0f
#define SPD_KI       500.0f
#define SPD_KD         0.0f
#define SPD_OUT_MAX 5000.0f       /* 电流限幅 */
#define SPD_I_MAX   5000.0f       /* 积分限幅（抗饱和） */

/* ================= 采样与滤波 ================= */
#define SPD_DT_S      0.001f      /* 1 ms */
#define SPD_ALPHA_LP  0.27f       /* 50 Hz 一阶低通（微分用） */

/* ================= 初始化 ================= */
void Speed_Init(void)
{
    PID_Init(&pid_spd,
             SPD_KP, SPD_KI, SPD_KD,
             SPD_DT_S,
             SPD_OUT_MAX,
             SPD_I_MAX,
             SPD_ALPHA_LP);

    pid_spd.err_wrap = 0;         /* 转速不需要 +-180 度归一化 */
    g_spd_target     = 0.0f;
}

/* ================= 设目标（清积分，防旧积分拖后腿） =================
 * 用于：上电使能、切换模式等"从零开始"的场合 */
void Speed_Set_Target(float rpm)
{
    g_spd_target = rpm;
    PID_Reset(&pid_spd, g_omega_rpm);    /* 用当前转速建微分基准 */
}

/* ================= 设目标（保留积分，用于阶跃） =================
 * 为什么不清积分：稳态时积分已经攒到"维持当前转速所需的电流"，
 * 阶跃时它正是新稳态需要的基础。清零会让电流瞬间掉到只剩比例项，
 * 转速先往下掉，再慢慢爬上来（实测 100→150 阶跃时下掉约 26 rpm）。 */
void Speed_Set_Target_KeepI(float rpm)
{
    g_spd_target = rpm;
}

/* ================= 核心：给定转速目标算电流 ================= */
float Speed_Calc(float rpm_target)
{
    
    float iq = PID_Calc(&pid_spd, rpm_target, g_omega_rpm);

    if (iq >  SPD_OUT_MAX) iq =  SPD_OUT_MAX;    /* 二次限幅 */
    if (iq < -SPD_OUT_MAX) iq = -SPD_OUT_MAX;

    return iq;
}

/* ================= 独立速度模式入口 ================= */
void Speed_Update(void)
{
    Motor_Set_Cmd((int16_t)Speed_Calc(g_spd_target));
}
