/**
  ******************************************************************************
  * @file    control.h
  * @brief   控制层：位置环（F-06）
  * @note    本层负责"用什么对象、什么参数、什么目标"，PID 算法本身在 pid.c
  ******************************************************************************
  */
#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"
#include "pid.h"

/* ================= 开环采集开关（编译期） =================
 * 1 = 开环：1 kHz 中断里【不调用】POS_P_UPDATE()，控制层完全不碰输出，
 *     由 main 里直接调用 Motor_Set_Cmd() 驱动
 *     ★ F-04 / F-05 采集时必须为 1
 * 0 = 闭环：正常调用控制层
 */
#define CTRL_OPENLOOP    0

/* ---- 控制层实例与对外变量 ---- */
extern PID_T             pid_pos;        /* 位置环控制器 */
extern volatile float    g_pos_target;   /* 目标角度（连续角，度） */
extern volatile uint8_t  g_ctrl_en;      /* 0=失能(输出0)，1=使能 */
extern volatile uint8_t  g_ctrl_phase;   /* 启动流程阶段：0=等待 1=保持 2=阶跃后 */
typedef enum {
  FAULT_NONE = 0,
  FAULT_CAN_TIMEOUT,
} fault_t;

extern volatile uint8_t g_fault;
/* ---- 接口 ---- */
void Control_Init(void);         /* 上电初始化（参数 + 失能） */
void POS_Set_Target(float deg);  /* 设置目标并清积分 */
void POS_P_UPDATE(void);         /* 1 kHz 中断里调用 */
void Control_Task(void);         /* 主循环里调用（启动/阶跃测试逻辑） */

#endif /* __CONTROL_H */
