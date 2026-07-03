#include "dm_motor.h"
#include "arm.h"
#include "cmsis_os.h"

#define ARM_CAN         hcan2
#define ARM_SPEED_UP    10.0f   /* 抬起速度 */
#define ARM_SPEED_DOWN  10.0f   /* 放下速度 */

/* 关节限幅 */
#define L_ROOT_MIN  -1.57f
#define L_ROOT_MAX   0.0f
#define L_TIP_MIN   -1.57f
#define L_TIP_MAX    0.0f
#define R_ROOT_MIN   0.0f
#define R_ROOT_MAX   1.57f
#define R_TIP_MIN    0.0f
#define R_TIP_MAX    1.57f

/* 目标位置 */
#define L_ROOT_UP    -1.57f   /* 左根竖起 */
#define L_ROOT_DOWN   0.0f    /* 左根水平 */
#define L_TIP_DOWN   -1.57f   /* 左末朝地 (暂未使用) */
#define L_TIP_UP_FWD -0.785f  /* 左末45° (竖起时) */
#define R_ROOT_UP     1.57f   /* 右根竖起 */
#define R_ROOT_DOWN   0.0f    /* 右根水平 */
#define R_TIP_DOWN    1.57f   /* 右末朝地 (暂未使用) */
#define R_TIP_UP_FWD  0.785f  /* 右末45° (竖起时) */

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

void arm_update(SBUS_t *sbus, bool select_left, bool tip_up_fwd)
{
    /* CH7 边沿 → 切换选中臂状态 */
    uint32_t now = xTaskGetTickCount();
    bool ch15 = (sbus->ch[15] < 650);   /* CH15回弹拨杆 */

    /* 首次调用 或 距上次超过50ms → 仅同步不切换 */
    if(g_arm_last_tick == 0 || (now - g_arm_last_tick) > pdMS_TO_TICKS(50))
    {
        g_last_ch7 = ch15;
        g_arm_last_tick = now;
    }
    /* 回弹拨杆: 按下+松开=一次边沿, 在松开时触发 */
    else if(ch15 && !g_last_ch7)
    {
        g_last_ch7 = ch15;
        g_arm_last_tick = now;
        if(select_left)
            g_arm_L = (g_arm_L == ARM_UP) ? ARM_DOWN : ARM_UP;
        else
            g_arm_R = (g_arm_R == ARM_UP) ? ARM_DOWN : ARM_UP;
    }
    else
    {
        g_last_ch7 = ch15;
        g_arm_last_tick = now;
    }

    /* 左臂目标 */
    if(g_arm_L == ARM_DOWN)
    {
        g_L_root = L_ROOT_DOWN;
        g_L_tip  = 0.0f;           /* 放下→末端朝前 */
    }
    else  /* ARM_UP */
    {
        g_L_root = L_ROOT_UP;
        g_L_tip  = tip_up_fwd ? L_TIP_UP_FWD : L_TIP_DOWN;  /* 吸45° / 放朝前 */
    }

    /* 右臂目标 */
    if(g_arm_R == ARM_DOWN)
    {
        g_R_root = R_ROOT_DOWN;
        g_R_tip  = 0.0f;           /* 放下→末端朝前 */
    }
    else
    {
        g_R_root = R_ROOT_UP;
        g_R_tip  = tip_up_fwd ? R_TIP_UP_FWD : R_TIP_DOWN;
    }
}

/* ================================================================ */

bool arm_is_left_down(void)  { return g_arm_L == ARM_DOWN; }
bool arm_is_right_down(void) { return g_arm_R == ARM_DOWN; }

/* ================================================================ */

void arm_task(void *parameter)
{
    while(1)
    {
        if(g_sys_enabled)
        {
            float r, spd;
            /* 左臂: ID1根, ID2末 */
            spd = (g_arm_L == ARM_DOWN) ? ARM_SPEED_DOWN : ARM_SPEED_UP;
            r = clampf(g_L_root, L_ROOT_MIN, L_ROOT_MAX);
            dm_pos_ctrl(&ARM_CAN, 1, r, spd); vTaskDelay(2);
            r = clampf(g_L_tip, L_TIP_MIN, L_TIP_MAX);
            dm_pos_ctrl(&ARM_CAN, 2, r, spd); vTaskDelay(2);
            /* 右臂: ID3根, ID4末 */
            spd = (g_arm_R == ARM_DOWN) ? ARM_SPEED_DOWN : ARM_SPEED_UP;
            r = clampf(g_R_root, R_ROOT_MIN, R_ROOT_MAX);
            dm_pos_ctrl(&ARM_CAN, 3, r, spd); vTaskDelay(2);
            r = clampf(g_R_tip, R_TIP_MIN, R_TIP_MAX);
            dm_pos_ctrl(&ARM_CAN, 4, r, spd);
        }
        vTaskDelay(20);
    }
}
