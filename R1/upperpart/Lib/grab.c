#include "dm_motor.h"
#include "grab.h"
#include "solenoid_valves.h"
#include "sbus_set.h"
#include "cmsis_os.h"

#define LIFT_CAN    hcan1
#define RACK_SPEED  6.0f           /* 2325齿条 */
#define FLIP_DOWN_SPEED  6.0f      /* 4310翻下去 */
#define FLIP_UP_SPEED    2.0f      /* 4310翻上来 */

static motor_t grab_motor[2];       /* [0]=4310翻转 ID5, [1]=2325齿条 ID6 */

/* ---- 工序状态机 ---- */
typedef enum {
    G_IDLE = 0,      /* 空闲 */
    G_RACK_ZERO,     /* 1: 齿条回零 */
    G_CLAW_OPEN,     /* 2: 开夹爪 */
    G_FLIP_DOWN,     /* 3: 翻转取杆 */
    G_CLAW_CLOSE,    /* 4: 关夹爪抓杆 */
    G_FLIP_BACK,     /* 5: 翻回 */
    G_MANUAL,        /* 6+: CH10手动控制齿条 */
} grab_step_t;

static grab_step_t g_step = G_IDLE;
static bool g_last_ch7 = false;
static bool g_first    = true;      /* 首次进入同步 */
static float g_rack = 0.0f;         /* 2325齿条目标 */
static float g_flip = 0.0f;         /* 4310翻转目标 */
static float g_flip_speed = FLIP_UP_SPEED;

extern bool g_sys_enabled;

/* ================================================================ */

void grab_init(void)
{
    dm_init(&grab_motor[0], 5, DM_MODE_POS, DM_4310);
    dm_init(&grab_motor[1], 6, DM_MODE_POS, DM_2325);
}

void grab_enable(void)
{
    dm_enable(&LIFT_CAN, &grab_motor[0]); vTaskDelay(2);
    dm_enable(&LIFT_CAN, &grab_motor[1]); vTaskDelay(2);
}

void grab_disable(void)
{
    dm_disable(&LIFT_CAN, &grab_motor[0]); vTaskDelay(2);
    dm_disable(&LIFT_CAN, &grab_motor[1]); vTaskDelay(2);
}

void grab_reset(void)
{
    g_step  = G_IDLE;
    g_rack  = 0.0f;
    g_flip  = 0.0f;
    YV3(0);
    g_first = true;      /* 下次 grab_update 时重新同步 CH7 */
}

/* ================================================================ */

void grab_update(SBUS_t *sbus)
{
    /* CH7 边沿 → 推进工序 */
    bool ch7 = (sbus->ch[7] < 650);

    if(g_first) { g_first = false; g_last_ch7 = ch7; }  /* 首帧仅同步 */
    else if(ch7 != g_last_ch7)
    {
        g_last_ch7 = ch7;
        if(g_step < G_MANUAL) g_step++;
    }

    /* CH10 始终控制齿条 */
    g_rack = Map((float)sbus->ch[10], 326.0f, 1659.0f, 0.0f, 12.0f);

    switch(g_step)
    {
        case G_IDLE:
            g_flip = 0.0f; YV3(0); g_flip_speed = FLIP_UP_SPEED;  break;

        case G_RACK_ZERO:
            g_flip = 0.0f; YV3(0); g_flip_speed = FLIP_UP_SPEED;  break;

        case G_CLAW_OPEN:
            YV3(1); break;

        case G_FLIP_DOWN:
            g_flip = -1.9f; g_flip_speed = FLIP_DOWN_SPEED;  break;

        case G_CLAW_CLOSE:
            YV3(0); break;

        case G_FLIP_BACK:
            g_flip = 0.0f; g_flip_speed = FLIP_UP_SPEED;  break;

        default:  /* G_MANUAL+ */
            break;
    }
}

/* ================================================================ */

void grab_task(void *parameter)
{
    while(1)
    {
        if(g_sys_enabled)
        {
            dm_pos_ctrl(&LIFT_CAN, 5, g_flip, g_flip_speed); vTaskDelay(2);
            dm_pos_ctrl(&LIFT_CAN, 6, g_rack, RACK_SPEED);
        }
        vTaskDelay(20);
    }
}
