/**
  ******************************************************************************
  * @file    motor_6623.h
  * @brief   6623 设备层：帧打包 / 帧解析 / 通信统计
  * @note    代码从 can.c 与 main.c 原样转移，未新增功能
  ******************************************************************************
  */
#ifndef __MOTOR_6623_H
#define __MOTOR_6623_H

#include "main.h"

/* ---- 解析结果（中断里写、主循环读）---- */
extern volatile uint16_t g_angle_raw;
extern volatile int16_t  g_current;
extern volatile int8_t   g_temp;
extern volatile int16_t  g_temp_raw;
extern uint8_t           g_raw[8];

/* ---- 通信统计 ---- */
extern volatile uint32_t g_rx_cnt;
extern volatile uint32_t g_rx_err;
extern volatile uint32_t g_tx_ok;
extern volatile uint32_t g_tx_fail;
extern volatile uint32_t g_tx_1k;

/* ---- 发送缓冲 ---- */
extern uint8_t g_tx[8];

/* ---- 接口 ---- */
void Motor_Set_Cmd(int16_t cmd);

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hc);

/* ---- 开环测试序列（F-05）----
 * 用法：control.h 里把 CTRL_OPENLOOP 设 1，主循环里调用 Test_play()。
 *       它按序发送一串电流阶跃（每档自带保持时间），跑完自动归零。
 * 用途：辨识开环增益 w/u、时间常数、摩擦死区。 */
void Test_play(void);            /* 主循环调用：按序发电流阶跃（非阻塞）*/

#endif /* __MOTOR_6623_H */
