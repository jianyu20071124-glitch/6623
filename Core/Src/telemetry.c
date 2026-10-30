/**
  ******************************************************************************
  * @file    telemetry.c
  * @brief   应用层：调试遥测
  * @note    0x302：1 kHz 角度/速度（T-06 等曲线的证据，满足"采样 ≥1 kHz"）
  *          0x300：100 ms 状态监视（原始角度、电流、温度、收发计数）
  ******************************************************************************
  */
#include "telemetry.h"
#include "can.h"
#include "motor_6623.h"
#include "angle.h"

uint8_t g_fb[8];
uint8_t g_dbg[8];

void can_echo(void) {
  static uint32_t last_fast = 0;
  static uint32_t last_slow = 0;
  static uint32_t slow_div  = 0;
  uint32_t now = HAL_GetTick();

  /* ---------- 0x302：1 kHz 角度/速度回显 ----------
     [0:1] angle_deg*100   [2:3] omega_rpm*100   [4:7] angle_cont*100
     （单位 0.01，T-06 曲线与 ±2% 带判定用） */
  if (now - last_fast >= 1)
  {
    last_fast = now;

    int16_t deg100 = (int16_t)(g_angle_deg * 100.0f);
    int16_t w100   = (int16_t)(g_omega_rpm * 100.0f);
    int32_t c100   = (int32_t)(g_angle_cont * 100.0f);

    g_dbg[0] = (uint8_t)((uint16_t)deg100 >> 8);
    g_dbg[1] = (uint8_t)((uint16_t)deg100 & 0xFF);
    g_dbg[2] = (uint8_t)((uint16_t)w100 >> 8);
    g_dbg[3] = (uint8_t)((uint16_t)w100 & 0xFF);
    g_dbg[4] = (uint8_t)((uint32_t)c100 >> 24);
    g_dbg[5] = (uint8_t)((uint32_t)c100 >> 16);
    g_dbg[6] = (uint8_t)((uint32_t)c100 >> 8);
    g_dbg[7] = (uint8_t)((uint32_t)c100 & 0xFF);

    CAN_Send(0x302u, g_dbg);

    /* ---------- 0x300：每 100 次（约 100 ms）发一次状态 ---------- */
    if (++slow_div >= 100u)
    {
      slow_div  = 0;
      last_slow = now;

      g_fb[0] = (uint8_t)(g_angle_raw >> 8);
      g_fb[1] = (uint8_t)(g_angle_raw & 0xFF);
      g_fb[2] = (uint8_t)((uint16_t)g_current >> 8);
      g_fb[3] = (uint8_t)((uint16_t)g_current & 0xFF);
      g_fb[4] = (uint8_t)g_temp;
      g_fb[5] = (uint8_t)g_rx_cnt;
      g_fb[6] = (uint8_t)g_rx_err;
      g_fb[7] = (uint8_t)g_tx_fail;

      CAN_Send(0x300u, g_fb);
    }
  }

  (void)last_slow;
}
