#include "dm_motor.h"
#include "arm.h"
#include "cmsis_os.h"

#define ARM_CAN    hcan2
#define ARM_SPEED  0.5f

/* 限幅 */
#define L_ROOT_MIN  -1.67f
#define L_ROOT_MAX   0.0f
#define L_TIP_MIN   -1.57f
#define L_TIP_MAX    0.0f
#define R_ROOT_MIN   0.0f
#define R_ROOT_MAX   1.67f
#define R_TIP_MIN    0.0f
#define R_TIP_MAX    1.57f

static motor_t arm_motor[4];    /* [0]左根4340 [1]左末4310 [2]右根4340 [3]右末4310 */

/* ---- 状态机 ---- */
typedef enum { ARM_UP, ARM_DOWN } arm_state_t;
static arm_state_t g_arm_L = ARM_UP;   /* 左臂 */
static arm_state_t g_arm_R = ARM_UP;   /* 右臂 */

/* 目标值 */
static float g_L_root = -1.67f, g_L_tip = 0.0f;
static float g_R_root =  1.67f, g_R_tip = 0.0f;

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
        g_L_root = 0.0f;
        g_L_tip  = ch6_high ? 0.0f : -1.57f;   /* 3层末端向前, 1~2层朝地 */
    }
    else  /* ARM_UP */
    {
        g_L_root = -1.67f;
        g_L_tip  = 0.0f;
    }

    /* 右臂目标 */
    if(g_arm_R == ARM_DOWN)
    {
        g_R_root = 0.0f;
        g_R_tip  = ch6_high ? 0.0f : 1.57f;
    }
    else
    {
        g_R_root = 1.67f;
        g_R_tip  = 0.0f;
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
