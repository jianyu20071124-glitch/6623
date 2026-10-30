/**
  ******************************************************************************
  * @file    cascade.h
  * @brief   位置-速度串级控制（F-06 / F-07 主力方案）
  * @note    外环：位置误差 -> 转速目标(rpm)
  *          内环：转速误差 -> 电流指令(count)
  ******************************************************************************
  */
#ifndef __CASCADE_H
#define __CASCADE_H

#include "main.h"
#include "pid.h"

/* ================= 外环（位置环）参数 ================= */
#define CASC_POS_KP       8.0f     /* rpm/° */
#define CASC_POS_KI       8.0f     /* rpm/(°·s) */
#define CASC_POS_KD       0.1f     /* rpm/(°/s) */
#define CASC_POS_OUT_MAX 100.0f    /* 转速限幅 rpm（限速）—— 放开到 600，前期满流才冲得动 */
#define CASC_POS_INT_MAX 100.0f    /* 外环积分限幅（rpm，与限速一致）*/

/* ================= 摩擦前馈 =================
 * 位置环最后 0.3° 补不上，是因为内环在亚 rpm 速度下被摩擦死区挡住。
 * 按转速指令方向叠加一个固定电流，帮它越过死区。
 * 实测：无前馈稳态 0.34°，有前馈 0.07° */
#define CASC_FF_POS     32.0f      /* 正向死区 26 × 1.15 */
#define CASC_FF_NEG     24.0f      /* 反向死区 16.4 × 1.3 */
#define CASC_FF_TH      0.12f      /* 死区判定阈值 rpm */

/* ================= 电流限幅 ================= */
#define CASC_IQ_MAX    5000.0f     /* count，与 speed.c 的 SPD_OUT_MAX 一致 */

/* ================= 对外变量 ================= */
extern volatile float g_rpm_target;   /* 速度环当前的目标转速 rpm（串口显示用）*/

/* ================= 接口 ================= */
void  CASC_Init(void);                          /* 初始化外环（内环由 speed.c 管）*/
float CASC_Calc(float target_deg, float meas_deg);  /* 返回电流 count */
PID_T *CASC_Get_PosPID(void);                   /* 取外环 PID（调参用）*/

#endif /* __CASCADE_H */
