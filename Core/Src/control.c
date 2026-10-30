/**
  ******************************************************************************
  * @file    control.c
  * @brief   控制层：模式调度 + 启动状态机
  * @note    本文件只做"选择用哪个控制器 + 什么时候给目标"，
  *          具体算法都在各自文件里：
  *            pos_lo.c  = 位置单环（位置 → 电流）
  *            cascade.c = 位置串级（位置 → 转速 → 电流）
  *            speed.c   = 速度环（转速 → 电流，被 cascade 当内环，也供速度单环用）
  *            sine.c    = 正弦目标发生器（给 cascade 用）
  ******************************************************************************
  */
#include "control.h"
#include "speed.h"
#include "pos_lo.h"
#include "cascade.h"
#include "motor_6623.h"
#include "angle.h"
#include "sine.h"
#include "step_seq.h"
#include <math.h>

/* ================= 控制模式选择（改这里，重新烧录生效） =================
 * 0 = 位置单环   位置误差 → 电流         （pos_lo.c）
 * 1 = 速度单环   转速误差 → 电流         （speed.c）
 * 2 = 正弦跟踪   正弦目标 + 位置串级     （sine.c + cascade.c）
 * 3 = 位置串级   位置误差 → 转速 → 电流  （cascade.c）
 */
#define CTRL_MODE_SEL    1
/* ================= 自动测试开关 =================
 * 0 = 不自动使能：上电后永远失能，电机自由转
 *     —— 采集 F-04（回绕/多圈）、F-05（开环辨识）时必须为 0
 * 1 = 自动跑：3 s 使能，再 2 s 做阶跃
 */
#define CTRL_AUTO_START  1

/* ================= 阶跃参数 ================= */
#define SPD_START_RPM  100.0f     /* 速度模式：阶跃前的起始转速 */
#define SPD_STEP_RPM   200.0f     /* 速度模式：阶跃后的目标转速 */
#define POS_STEP_DEG    30.0f     /* 位置模式：单次阶跃幅度（°）*/

/* ================= 阶跃序列开关 =================
 * 1 = 使能后每隔 1 秒自动做一次阶跃（步进_seq.c）：
 *       0 → +30 → 0 → −30 → 0 → +90 → 0 → −90 → 0 → +180 → 0 → −180 → 0 → 循环
 * 0 = 只做一次 POS_STEP_DEG 的阶跃（原来 T-06 的做法）
 * 仅对位置模式（0 / 2 / 3）有效；速度模式忽略。 */
#define POS_SEQ_ENABLE   1

/* ================= 时间参数 ================= */
#define CTRL_DT_S      0.001f     /* 1 ms 控制周期 */
#define START_DELAY_MS 3000u      /* 上电 3 s 后使能 */
#define STEP_DELAY_MS  2000u      /* 使能后 2 s 做阶跃 */

/* ================= 对外变量 ================= */
volatile float   g_pos_target = 0.0f;
volatile uint8_t g_ctrl_en    = 0;

/* ================= 上电初始化 ================= */
void Control_Init(void)
{
    PL_Init();       /* 位置单环（pos_lo.c，参数在 pos_lo.h）*/
    CASC_Init();     /* 位置串级外环（cascade.c，参数在 cascade.h）*/
    Speed_Init();    /* 速度环（speed.c，串级内环 + 速度单环都用它）*/
    Seq_Init();      /* 阶跃序列发生器（step_seq.c）*/

#if (POS_SEQ_ENABLE == 1)
    /* 序列模式：目标是"相对累加"，误差恒等于阶跃幅度（≤180°），
       不需要角度归一化。而且 180° 误差在归一化边界上符号不确定
       （实测 −180° 那一步会被当成 +180° 走反，导致后续全部偏 360°），
       所以序列模式关掉 err_wrap，让误差符号唯一。
       单次阶跃（POS_SEQ_ENABLE=0）时保留 err_wrap=1（pos_lo.c 里的默认值），
       这样跨 0/360° 时走最短路径。 */
    PL_Get_PID()->err_wrap      = 0;
    CASC_Get_PosPID()->err_wrap = 0;
#else
    PL_Get_PID()->err_wrap      = 1;
    CASC_Get_PosPID()->err_wrap = 1;
#endif

#if (CTRL_MODE_SEL == 2)
    Sine_Init();     /* 正弦发生器（sine.c，参数在 sine.h）*/
#endif

    g_ctrl_en    = 0;              /* 先失能，由 Control_Task 决定何时使能 */
    g_pos_target = g_angle_cont;   /* 目标先设为当前位置 */
}

/* ================= 设置位置目标（同时清所有积分） ================= */
void POS_Set_Target(float deg)
{
    g_pos_target = deg;

    PID_Reset(PL_Get_PID(),   g_angle_cont);   /* 清位置单环 */
    PID_Reset(CASC_Get_PosPID(), g_angle_cont);/* 清串级外环 */
    PID_Reset(&pid_spd,       g_omega_rpm);    /* 清速度内环 */
}

/* ================= 控制主循环：1 kHz 中断里调用 ================= */
void POS_P_UPDATE(void)
{
    /* 失能：立即输出 0，防失控 */
    if (!g_ctrl_en)
    {
        Motor_Set_Cmd(0);
        return;
    }

#if (CTRL_MODE_SEL == 2)
    /* 正弦模式：每 1 ms 更新目标角度（用串级去追）*/
    g_pos_target = Sine_Update();
#endif

    float iq = 0.0f;

#if (CTRL_MODE_SEL == 0)
    /* ---------- 位置单环：位置误差 → 电流 ----------
       单环没有速度环，不存在"速度目标"，这里置 0（串口 CH6 显示 0 属正常）*/
    g_rpm_target = 0.0f;
    iq = PL_Calc(g_pos_target, g_angle_cont);

#elif (CTRL_MODE_SEL == 1)
    /* ---------- 速度单环：转速误差 → 电流 ---------- */
    g_rpm_target = g_spd_target;
    iq = Speed_Calc(g_spd_target);

#else
    /* ---------- 位置串级（含正弦）：位置 → 转速 → 电流 ---------- */
    iq = CASC_Calc(g_pos_target, g_angle_cont);
#endif

    Motor_Set_Cmd((int16_t)iq);
}

/* ================= 启动流程状态 ================= */
typedef enum
{
    CTRL_PHASE_WAIT = 0,     /* 等待上电稳定（未使能） */
    CTRL_PHASE_HOLD = 1,     /* 已使能，目标=当前角度，保持不动 */
    CTRL_PHASE_STEP = 2      /* 已发出阶跃/已开始动作 */
} ctrl_phase_t;

volatile uint8_t g_ctrl_phase = CTRL_PHASE_WAIT;

/* ================= 主循环任务：状态机 ================= */
void Control_Task(void)
{
#if (CTRL_AUTO_START == 0)
    /* 采集模式：不自动使能，电机保持自由（F-04 / F-05 用） */
    return;
#else
    static uint32_t s_t_enable = 0;      /* 进入 HOLD 的时刻 */

    switch (g_ctrl_phase)
    {
    /* ---------- 阶段 0：等待，然后使能 ---------- */
    case CTRL_PHASE_WAIT:
        /* 条件：上电满 3 s，且确实收到过 CAN 反馈（连续角有效） */
        if ((HAL_GetTick() > START_DELAY_MS) && (g_rx_cnt > 100u))
        {
#if (CTRL_MODE_SEL == 1)
            /* 速度模式：先稳在起始转速 */
            g_spd_target = SPD_START_RPM;
            PID_Reset(&pid_spd, g_omega_rpm);
#else
            /* 位置模式：目标 = 当前角度，清所有积分 */
            g_pos_target = g_angle_cont;
            PID_Reset(PL_Get_PID(),      g_angle_cont);
            PID_Reset(CASC_Get_PosPID(), g_angle_cont);
            PID_Reset(&pid_spd,          g_omega_rpm);
#endif
            g_ctrl_en    = 1;                 /* 开闸 */
            s_t_enable   = HAL_GetTick();
            g_ctrl_phase = CTRL_PHASE_HOLD;   /* 进入下一阶段 */
        }
        break;

    /* ---------- 阶段 1：保持 2 s，然后开始动作 ---------- */
    case CTRL_PHASE_HOLD:
        if ((HAL_GetTick() - s_t_enable) > STEP_DELAY_MS)
        {
#if (CTRL_MODE_SEL == 2)
            Sine_Start(g_angle_cont);                    /* 开始正弦摆动 */
#elif (CTRL_MODE_SEL == 1)
            Speed_Set_Target_KeepI(SPD_STEP_RPM);        /* 速度阶跃（保留积分）*/
#else
#if (POS_SEQ_ENABLE == 1)
            /* 序列模式：以当前位置为中心，之后每 1 s 自动阶跃一次 */
            Seq_Start(g_angle_cont);
#else
            POS_Set_Target(g_pos_target + POS_STEP_DEG); /* 位置阶跃（单次）*/
#endif
#endif
            g_ctrl_phase = CTRL_PHASE_STEP;
        }
        break;

    /* ---------- 阶段 2：保持 ---------- */
    case CTRL_PHASE_STEP:
#if (POS_SEQ_ENABLE == 1) && (CTRL_MODE_SEL != 1) && (CTRL_MODE_SEL != 2)
        /* 序列推进：每 1 秒给一个新目标，并清掉积分（每次阶跃都从零开始，
           避免上一次的积分带到下一次，指标才可比）*/
        if (Seq_Update())
        {
            POS_Set_Target(Seq_Get_Target());
        }
#endif
        break;
    default:
        break;
    }
#endif /* CTRL_AUTO_START */
}
