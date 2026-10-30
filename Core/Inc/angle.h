/**
  ******************************************************************************
  * @file    angle.h
  * @brief   控制层：角度解算与速度估计（F-04）
  ******************************************************************************
  */
#ifndef __ANGLE_H
#define __ANGLE_H

#include "main.h"

/* ---- 解算结果 ---- */
extern volatile float   g_angle_deg;     /* 单圈角度 0~360° */
extern volatile float   g_angle_cont;
extern volatile float   g_omega_rpm;     /* 角速度 rpm（一阶低通后，控制用） */
extern volatile float   g_omega_iir_rpm; /* 角速度 rpm（一阶低通，α=0.27，fc≈50Hz） */
extern volatile float   g_omega_raw_rpm; /* 角速度 rpm（未滤波，仅用于测量滤波延迟） */
extern volatile int32_t  g_turns;       /* 多圈计数 */

/* 在 1 kHz 定时器里调用 */
void Angle_Update(void);

#endif /* __ANGLE_H */
