#include "dm_motor.h"
#include "arm.h"
#include "cmsis_os.h"

#define ARM_CAN    hcan2
#define ARM_SPEED  0.5f

/* 关节限幅 */
#define L_ROOT_MIN  -1.66f
#define L_ROOT_MAX   0.0f
#define L_TIP_MIN   -1.57f
#define L_TIP_MAX    0.0f
#define R_ROOT_MIN   0.0f
#define R_ROOT_MAX   1.66f
#define R_TIP_MIN    0.0f
#define R_TIP_MAX    1.57f

/* 目标位置 */
#define L_ROOT_UP    -1.66f   /* 左根竖起 */
#define L_ROOT_DOWN   0.0f    /* 左根水平 */
#define L_TIP_UP      0.0f    /* 左末顺臂 */
#define L_TIP_DOWN   -1.57f   /* 左末朝地 (1~2层) */
#define R_ROOT_UP     1.66f   /* 右根竖起 */
#define R_ROOT_DOWN   0.0f    /* 右根水平 */
#define R_TIP_UP      0.0f    /* 右末顺臂 */
#define R_TIP_DOWN    1.57f   /* 右末朝地 (1~2层) */

static motor_t arm_motor[4];    /* [0]左根4340 [1]左末4310 [2]右根4340 [3]右末4310 */

/* ---- 状态机 ---- */
typedef enum { ARM_UP, ARM_DOWN } arm_state_t;
static arm_state_t g_arm_L = ARM_UP;   /* 左臂 */
static arm_state_t g_arm_R = ARM_UP;   /* 右臂 */

/* 目标值 */
static float g_L_root = L_ROOT_UP, g_L_tip = L_TIP_UP;
static float g_R_root = R_ROOT_UP, g_R_tip = R_TIP_UP;

static bool g_last_ch7 = false;        /* CH7 边沿 */
static bool g_first  = true;            /* 首次进入, 同步不切换 */

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
    g_first = true;   /* 下次 arm_update 时同步 CH7 */
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

void arm_update(SBUS_t *sbus, bool select_left, bool ch6_high)
{
    /* CH7 边沿 → 切换选中臂状态 */
    bool ch7 = (sbus->ch[7] < 650);     /* 物理上拨=ch_low */

    if(g_first) { g_first = false; g_last_ch7 = ch7; }  /* 首帧仅同步, 不切换 */
    else if(ch7 != g_last_ch7)
    {
        g_last_ch7 = ch7;
        if(select_left)
            g_arm_L = (g_arm_L == ARM_UP) ? ARM_DOWN : ARM_UP;
        else
            g_arm_R = (g_arm_R == ARM_UP) ? ARM_DOWN : ARM_UP;
    }

    /* 左臂目标 */
    if(g_arm_L == ARM_DOWN)
    {
        g_L_root = L_ROOT_DOWN;
        g_L_tip  = ch6_high ? L_TIP_UP : L_TIP_DOWN; /* 3层向前, 1~2层朝地 */
    }
    else  /* ARM_UP */
    {
        g_L_root = L_ROOT_UP;
        g_L_tip  = L_TIP_UP;
    }

    /* 右臂目标 */
    if(g_arm_R == ARM_DOWN)
    {
        g_R_root = R_ROOT_DOWN;
        g_R_tip  = ch6_high ? R_TIP_UP : R_TIP_DOWN;
    }
    else
    {
        g_R_root = R_ROOT_UP;
        g_R_tip  = R_TIP_UP;
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
