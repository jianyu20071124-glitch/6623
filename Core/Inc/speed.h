/**
  ******************************************************************************
  * @file    speed.h
  * @brief   速度环（F-07）：转速目标 -> 电流指令
  ******************************************************************************
  */
#ifndef __SPEED_H
#define __SPEED_H

#include "main.h"
#include "pid.h"

/* ---- 实例与目标 ---- */
extern PID_T           pid_spd;          /* 速度环控制器 */
extern volatile float  g_spd_target;     /* 速度目标（rpm） */

/* ---- 接口 ---- */
void  Speed_Init(void);                  /* 上电初始化 */
void  Speed_Set_Target(float rpm);       /* 设目标（清积分，用于使能/换模式） */
void  Speed_Set_Target_KeepI(float rpm); /* 设目标（保留积分，用于阶跃） */
float Speed_Calc(float rpm_target);      /* 给定目标 -> 返回电流 count */
void  Speed_Update(void);                /* 用 g_spd_target 直接控制 */

#endif /* __SPEED_H */
