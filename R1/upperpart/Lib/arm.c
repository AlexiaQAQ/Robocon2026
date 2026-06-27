#include "dm_motor.h"
#include "arm.h"
#include "cmsis_os.h"

#define ARM_CAN    hcan2
#define ARM_SPEED  0.5f

/* 关节限幅 */
#define L_ROOT_MIN  -1.57f
#define L_ROOT_MAX   0.0f
#define L_TIP_MIN   -1.57f
#define L_TIP_MAX    1.57f
#define R_ROOT_MIN   0.0f
#define R_ROOT_MAX   1.57f
#define R_TIP_MIN   -1.57f
#define R_TIP_MAX    1.57f

/* 目标位置 */
#define L_ROOT_UP    -1.57f   /* 左根竖起 */
#define L_ROOT_DOWN   0.0f    /* 左根水平 */
#define L_TIP_UP      0.0f    /* 左末顺臂 (臂放平时朝前) */
#define L_TIP_DOWN   -1.57f   /* 左末朝地 */
#define L_TIP_UP_FWD -1.57f   /* 左末竖起朝前 */
#define L_TIP_UP_45  -0.785f  /* 左末竖起45°兜方块 */
#define R_ROOT_UP     1.57f   /* 右根竖起 */
#define R_ROOT_DOWN   0.0f    /* 右根水平 */
#define R_TIP_UP      0.0f    /* 右末顺臂 (臂放平时朝前) */
#define R_TIP_DOWN    1.57f   /* 右末朝地 */
#define R_TIP_UP_FWD  1.57f   /* 右末竖起朝前 */
#define R_TIP_UP_45   0.785f  /* 右末竖起45°兜方块 */

static motor_t arm_motor[4];    /* [0]左根4340 [1]左末4310 [2]右根4340 [3]右末4310 */

/* ---- 状态机 ---- */
typedef enum { ARM_UP, ARM_DOWN } arm_state_t;
static arm_state_t g_arm_L = ARM_UP;   /* 左臂 */
static arm_state_t g_arm_R = ARM_UP;   /* 右臂 */

/* 目标值 */
static float g_L_root = L_ROOT_UP, g_L_tip = L_TIP_UP_FWD;
static float g_R_root = R_ROOT_UP, g_R_tip = R_TIP_UP_FWD;

static bool     g_last_ch7      = false; /* CH7 边沿 */
static uint32_t g_arm_last_tick = 0;     /* 上次调用tick, 检测跨模式断档 */

extern bool g_sys_enabled;

/* ---- 辅助 ---- */
static inline float clampf(float v, float lo, float hi)
{
    if(v < lo) return lo;
    if(v > hi) return hi;
    return v;
}

/* ================================================================ */

void arm_init(void)
{
    dm_init(&arm_motor[0], 1, DM_MODE_POS, DM_4340);
    dm_init(&arm_motor[1], 2, DM_MODE_POS, DM_4310);
    dm_init(&arm_motor[2], 3, DM_MODE_POS, DM_4340);
    dm_init(&arm_motor[3], 4, DM_MODE_POS, DM_4310);
}

void arm_enable(void)
{
    dm_enable(&ARM_CAN, &arm_motor[0]); vTaskDelay(2);
    dm_enable(&ARM_CAN, &arm_motor[1]); vTaskDelay(2);
    dm_enable(&ARM_CAN, &arm_motor[2]); vTaskDelay(2);
    dm_enable(&ARM_CAN, &arm_motor[3]); vTaskDelay(2);
}

void arm_disable(void)
{
    dm_disable(&ARM_CAN, &arm_motor[0]); vTaskDelay(2);
    dm_disable(&ARM_CAN, &arm_motor[1]); vTaskDelay(2);
    dm_disable(&ARM_CAN, &arm_motor[2]); vTaskDelay(2);
    dm_disable(&ARM_CAN, &arm_motor[3]); vTaskDelay(2);
}

/* ================================================================ */

void arm_update(SBUS_t *sbus, bool select_left, bool tip_45)
{
    /* CH7 边沿 → 切换选中臂状态 */
    uint32_t now = xTaskGetTickCount();
    bool ch7 = (sbus->ch[7] < 650);     /* 物理上拨=ch_low */

    /* 首次调用 或 距上次超过50ms → 仅同步不切换 */
    if(g_arm_last_tick == 0 || (now - g_arm_last_tick) > pdMS_TO_TICKS(50))
    {
        g_last_ch7 = ch7;
        g_arm_last_tick = now;
    }
    else if(ch7 != g_last_ch7)
    {
        g_last_ch7 = ch7;
        g_arm_last_tick = now;
        if(select_left)
            g_arm_L = (g_arm_L == ARM_UP) ? ARM_DOWN : ARM_UP;
        else
            g_arm_R = (g_arm_R == ARM_UP) ? ARM_DOWN : ARM_UP;
    }
    else
    {
        g_arm_last_tick = now;
    }

    /* 左臂目标 */
    if(g_arm_L == ARM_DOWN)
    {
        g_L_root = L_ROOT_DOWN;
        g_L_tip  = L_TIP_UP;                     /* 放下→末端朝前 */
    }
    else  /* ARM_UP */
    {
        g_L_root = L_ROOT_UP;
        g_L_tip  = tip_45 ? L_TIP_UP_FWD : L_TIP_UP_45;  /* 朝前 / 45°兜 */
    }

    /* 右臂目标 */
    if(g_arm_R == ARM_DOWN)
    {
        g_R_root = R_ROOT_DOWN;
        g_R_tip  = R_TIP_UP;                     /* 放下→末端朝前 */
    }
    else
    {
        g_R_root = R_ROOT_UP;
        g_R_tip  = tip_45 ? R_TIP_UP_FWD : R_TIP_UP_45;  /* 朝前 / 45°兜 */
    }
}

/* ================================================================ */

void arm_task(void *parameter)
{
    while(1)
    {
        if(g_sys_enabled)
        {
            float r;
            /* 左根: ID1, 左末: ID2 */
            r = clampf(g_L_root, L_ROOT_MIN, L_ROOT_MAX);
            dm_pos_ctrl(&ARM_CAN, 1, r, ARM_SPEED); vTaskDelay(2);
            r = clampf(g_L_tip, L_TIP_MIN, L_TIP_MAX);
            dm_pos_ctrl(&ARM_CAN, 2, r, ARM_SPEED); vTaskDelay(2);
            /* 右根: ID3, 右末: ID4 */
            r = clampf(g_R_root, R_ROOT_MIN, R_ROOT_MAX);
            dm_pos_ctrl(&ARM_CAN, 3, r, ARM_SPEED); vTaskDelay(2);
            r = clampf(g_R_tip, R_TIP_MIN, R_TIP_MAX);
            dm_pos_ctrl(&ARM_CAN, 4, r, ARM_SPEED);
        }
        vTaskDelay(20);
    }
}
