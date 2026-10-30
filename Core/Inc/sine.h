/**
  ******************************************************************************
  * @file    sine.h
  * @brief   正弦跟踪发生器（F-08 / T-09）
  * @note    只负责产生"随时间变化的目标角度"，不涉及 PID。
  *          控制环仍然用现有的位置-速度串级，只是把 g_pos_target 交给它更新。
  ******************************************************************************
  */
#ifndef __SINE_H
#define __SINE_H

#include "main.h"

/* ================= 可调参数 ================= */
#define SINE_FREQ_HZ     0.5f      /* 频率：0.5 Hz（2 秒一个来回） */
#define SINE_AMP_DEG    30.0f      /* 摆幅：±30° */
#define SINE_DUR_MS   10000u       /* 持续时间：10 秒 */

/* ================= 接口 ================= */
void  Sine_Init(void);             /* 上电初始化（复位状态） */
void  Sine_Start(float center_deg);/* 从 center_deg 为中心开始摆动 */
void  Sine_Stop(void);             /* 停止摆动（目标回到中心） */
float Sine_Update(void);           /* 每 1 ms 调一次，返回当前目标角度 */
uint8_t Sine_IsRunning(void);      /* 1 = 正在摆动 */
float Sine_GetCenter(void);        /* 取摆动中心 */

#endif /* __SINE_H */
