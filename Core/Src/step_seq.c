/**
  ******************************************************************************
  * @file    step_seq.c
  * @brief   位置阶跃序列发生器（T-06 / T-07 批量测试用）
  * @note    每 1 秒走一步，幅度序列：+30, −30, +90, −90, +180, −180（循环）。
  *
  *          为什么用"相对累加"而不是绝对角度：
  *            g_angle_cont 是多圈连续角，如果写绝对目标（例如 center−180），
  *            位置环的 err_wrap 会把 −360° 的误差归一化成 0，电机原地不动。
  *            相对累加时每次的目标 = 当前目标 + 幅度，误差恒等于 ±幅度，
  *            不会超过 180°，方向唯一，物理上也正好是来回摆动。
  ******************************************************************************
  */
#include "step_seq.h"

/* ================= 幅度序列（相对上一步的增量，单位 °） ================= */
static const float s_seq[] =
{
    +30.0f,
    -30.0f,
    +90.0f,
    -90.0f,
   +180.0f,
   -180.0f,
};

#define SEQ_LEN   (sizeof(s_seq) / sizeof(s_seq[0]))

/* ================= 内部状态 ================= */
static float    s_center = 0.0f;    /* 起点角度（使能时的位置） */
static float    s_target = 0.0f;    /* 当前目标（绝对角度，累加得到） */
static uint16_t s_idx    = 0;       /* 当前步号 */
static uint32_t s_t0     = 0;       /* 本步开始时刻 */
static uint8_t  s_run    = 0;       /* 1 = 序列进行中 */

/* ================= 初始化 ================= */
void Seq_Init(void)
{
    s_center = 0.0f;
    s_target = 0.0f;
    s_idx    = 0;
    s_t0     = 0;
    s_run    = 0;
}

/* ================= 开始 ================= */
void Seq_Start(float center_deg)
{
    s_center = center_deg;
    s_target = center_deg;          /* 先停在起点，等 1 秒再走第一档 */
    s_idx    = 0;
    s_t0     = HAL_GetTick();
    s_run    = 1;
}

/* ================= 停止（保持当前目标，不再推进） ================= */
void Seq_Stop(void)
{
    s_run = 0;
}

/* ================= 每 1 秒推进一格 =================
 * 返回 1 表示"有了新目标，请调用方更新位置目标" */
uint8_t Seq_Update(void)
{
    if (!s_run)
    {
        return 0;
    }

    if ((HAL_GetTick() - s_t0) < SEQ_STEP_MS)
    {
        return 0;                       /* 还没到 1 秒 */
    }

    /* 到点了：目标累加一个幅度（相对上一步），步号循环 */
    s_t0     = HAL_GetTick();
    s_target += s_seq[s_idx];
    s_idx    = (uint16_t)((s_idx + 1u) % SEQ_LEN);

    return 1;
}

/* ================= 查询 ================= */
float Seq_Get_Target(void)
{
    return s_target;
}

uint8_t Seq_Is_Running(void)
{
    return s_run;
}

uint16_t Seq_Get_Index(void)
{
    return s_idx;
}
