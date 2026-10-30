/**
  ******************************************************************************
  * @file    motor_6623.c
  * @brief   6623 设备层实现
  * @note    变量定义与两个函数均从 can.c / main.c 原样转移，未新增功能
  ******************************************************************************
  */
#include "motor_6623.h"

/* ============ 变量定义（原 can.c USER CODE 0）============ */
volatile uint16_t g_angle_raw;
volatile int16_t  g_current;
volatile int8_t   g_temp;
volatile int16_t  g_temp_raw;
volatile uint32_t g_rx_cnt;
volatile uint32_t g_rx_err;
volatile uint32_t g_tx_ok;
volatile uint32_t g_tx_fail;
volatile uint32_t g_tx_1k;
uint8_t           g_tx[8];
uint8_t           g_raw[8];



void Motor_Set_Cmd(int16_t cmd) {
  if (cmd >  5000) cmd =  5000;
  if (cmd < -5000) cmd = -5000;

  g_tx[0] = (uint8_t)((uint16_t)cmd >> 8);
  g_tx[1] = (uint8_t)((uint16_t)cmd & 0xFFu);
  g_tx[2] = (uint8_t)((uint16_t)cmd >> 8);
  g_tx[3] = (uint8_t)((uint16_t)cmd & 0xFFu);
  g_tx[4] = 0;
  g_tx[5] = 0;
  g_tx[6] = 0x01;          /* 电流闭环 */
  g_tx[7] = 0;
}

/* ============ 开环测试序列（F-05）============
 * 用途：按序发一串电流阶跃，辨识开环增益 w/u、时间常数、摩擦死区。
 * 使用条件：control.h 的 CTRL_OPENLOOP = 1
 *           （此时 1 kHz 中断不调用 POS_P_UPDATE，控制层不会覆盖这里的命令）。
 * 每档自带保持时间（ms），跑完自动归零。
 */
typedef struct
{
    int16_t  cmd;    /* 目标电流（count）*/
    uint16_t ms;     /* 保持时间（ms）*/
} step_t;

static const step_t seq[] = {
    {    0, 1000},   /* 静止，建基线 */
    {  500, 2000},
    {    0, 1000},
    { 1000, 2000},
    {    0, 1000},
    { 2000, 2000},
    {    0, 2000},
    { -500, 2000},
    {    0, 1000},
    {-1000, 2000},
    {    0, 1000},
    {-2000, 2000},
    {    0, 3000},
};

#define TEST_N   (sizeof(seq) / sizeof(seq[0]))

static uint8_t  s_step = 0;      /* 当前档位 */
static uint32_t s_t0   = 0;      /* 本档开始时刻 */

void Test_play(void)
{
    /* 跑完：保持 0 输出 */
    if (s_step >= TEST_N)
    {
        Motor_Set_Cmd(0);
        return;
    }

    /* 输出本档电流 */
    Motor_Set_Cmd(seq[s_step].cmd);

    /* 到点则进下一档 */
    if ((HAL_GetTick() - s_t0) >= seq[s_step].ms)
    {
        s_t0 = HAL_GetTick();
        s_step++;
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hc)
{
  CAN_RxHeaderTypeDef rx;
  uint8_t d[8];

  if (HAL_CAN_GetRxMessage(hc, CAN_RX_FIFO1, &rx, d) != HAL_OK) { g_rx_err++; return; }
  if (((rx.StdId != 0x205u)&&(rx.StdId!=0x206u)) || (rx.DLC != 8u))                   { g_rx_err++; return; }
   for (int i = 0; i < 8; i++) g_raw[i] = d[i];

  g_angle_raw = (uint16_t)(8191u - (((uint16_t)d[0] << 8) | d[1]));/*转的时候raw在减小所以反过来*/
  g_current   = (int16_t) ((d[2] << 8) | d[3]);
  g_temp      = (int8_t)   d[6];

  g_rx_cnt++;

}
