/**
  ******************************************************************************
  * @file    pid.h
  * @brief   通用 PID 控制器（纯算法层，不含任何控制对象信息）
  * @note    位置式 + 微分作用于测量值 + 微分低通 + 条件积分抗饱和
  ******************************************************************************
  */
#ifndef __PID_H
#define __PID_H

#include "main.h"

typedef struct
{
    /* ---- 参数（调参时改） ---- */
    float kp;              /* 比例系数，单位随被控量而定（counts/单位） */
    float ki;              /* 积分系数，counts/(单位·s) */
    float kd;              /* 微分系数，counts/(单位/s) */
    float dt;              /* 采样周期，秒（必须 > 0） */

    /* ---- 限幅 ---- */
    float out_max, out_min;    /* 输出限幅 */
    float i_max, i_min;        /* 积分限幅（抗饱和） */

    /* ---- 微分低通 ---- */
    float d_alpha;         /* 一阶低通系数 (0,1)，50Hz/1ms → 0.27 */

    /* ---- 运行时状态 ---- */
    float integ;           /* 积分累加 */
    float prev_meas;       /* 上拍测量值 */
    float d_filt;          /* 滤波后的微分值 */
    float err;             /* 本拍误差（遥测用） */
    float out;             /* 本拍输出（遥测用） */

    /* ---- 标志 ---- */
    uint8_t primed;        /* 0=还没有基准（首拍不算微分） */
    uint8_t saturated;     /* 上拍输出是否饱和（条件积分用） */
    float i_zone;
    uint8_t err_wrap;      /* 1=误差归一化到 ±180°（角度环用） */
} PID_T;

/* ---- 接口 ---- */
void  PID_Init(PID_T *p, float kp, float ki, float kd, float dt,
               float out_max, float i_max, float d_alpha);
void  PID_Reset(PID_T *p, float meas);
float PID_Calc(PID_T *p, float target, float meas);

#endif /* __PID_H */
