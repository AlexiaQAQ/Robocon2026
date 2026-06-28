#include "dm_motor.h"
#include "grab.h"
#include "solenoid_valves.h"
#include "sbus_set.h"
#include "cmsis_os.h"

#define LIFT_CAN    hcan1
#define RACK_SPEED  6.0f           /* 2325齿条 */
#define FLIP_DOWN_SPEED  6.0f      /* 4310翻下去 */
#define FLIP_UP_SPEED    2.0f      /* 4310翻上来 */
#define FLIP_UP_POS      0.0f      /* 4310翻上位置 */
#define FLIP_DOWN_POS   -1.8f      /* 4310翻下取杆位置 */
#define FLIP_TOGGLE_POS -0.4f    /* 4310对接翻转位置 */
#define FLIP_MAX         0.0f      /* 4310翻转上限 */
#define FLIP_MIN        -2.0f      /* 4310翻转下限 */
#define RACK_ZERO_TOL    0.2f      /* 齿条回零容差, 开爪前检查 */
#define GRAB_H_HIGH      25.0f     /* 高位 (CH6上拨) */
#define GRAB_H_DOCK      14.15f    /* 对接 (CH6中位) */
#define LIFT_FINE_STEP    0.05f    /* CH1每次微调步长 */
#define LIFT_FINE_MAX     3.0f     /* CH1微调最大累积 */

static motor_t grab_motor[2];       /* [0]=4310翻转 ID5, [1]=2325齿条 ID6 */

/* ---- 工序状态机 ---- */
typedef enum {
    G_IDLE = 0,      /* 空闲 */
    G_RACK_ZERO,     /* 1: 齿条回零 */
    G_CLAW_OPEN,     /* 2: 开夹爪 */
    G_FLIP_DOWN,     /* 3: 翻转取杆 */
    G_CLAW_CLOSE,    /* 4: 关夹爪抓杆 */
    G_FLIP_BACK,     /* 5: 翻回 */
    G_LIFT_RISE,     /* 6: 对接/回零 + CH10手动, CH6切换高度 */
} grab_step_t;

static grab_step_t g_step = G_IDLE;
static bool     g_last_ch7       = false;  /* CH7 边沿 */
static uint32_t g_grab_last_tick = 0;      /* 上次调用tick, 检测跨模式断档 */
static float g_rack = 0.0f;         /* 2325齿条目标 */
static float g_flip = FLIP_UP_POS;   /* 4310翻转目标 */
static float g_flip_speed = FLIP_UP_SPEED;
static bool  g_flip_toggle = false;        /* 步骤6 CH7切换翻转位置 */
static float    g_ch1_offset     = 0.0f;  /* CH1摇杆微调偏移 */
static int      g_ch1_zone       = 0;      /* CH1当前区域: -1低/0中/1高 */
static uint32_t g_ch1_last_tick  = 0;      /* 上次调用tick, 检测跨模式断档 */

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
    g_step        = G_IDLE;
    g_rack        = 0.0f;
    g_flip        = FLIP_UP_POS;
    g_flip_toggle = false;
    g_ch1_offset  = 0.0f;
    YV3(0);
}

/* ================================================================ */

void grab_update(SBUS_t *sbus)
{
    /* CH7 边沿 → 推进工序 */
    uint32_t now = xTaskGetTickCount();
    bool ch7 = (sbus->ch[7] < 650);

    /* 首次调用 或 距上次超过50ms → 仅同步不推进 (防跨模式误触发) */
    if(g_grab_last_tick == 0 || (now - g_grab_last_tick) > pdMS_TO_TICKS(50))
    {
        g_last_ch7 = ch7;
        g_grab_last_tick = now;
    }
    else if(ch7 != g_last_ch7)
    {
        g_last_ch7 = ch7;
        g_grab_last_tick = now;
        if(g_step < G_LIFT_RISE) g_step++;    /* 0→5 线性推进 */
        else  g_flip_toggle = !g_flip_toggle; /* 步骤6: 切换翻转位置 */
    }
    else
    {
        g_grab_last_tick = now;
    }

    /* CH1 摇杆增量微调 (仅步骤6+生效, 边沿触发, 需回中才能再次调节) */
    if(g_step >= G_LIFT_RISE)
    {
        float ch1 = (float)sbus->ch[1];
        int zone = (ch1 > 1012.0f) ? 1 : (ch1 < 972.0f) ? -1 : 0;

        /* 首次调用 或 距上次超过50ms → 仅同步不调节 (防跨模式误触发) */
        if(g_ch1_last_tick == 0 || (xTaskGetTickCount() - g_ch1_last_tick) > pdMS_TO_TICKS(50))
        {
            g_ch1_zone = zone;
            g_ch1_last_tick = xTaskGetTickCount();
        }
        else if(zone != g_ch1_zone)
        {
            if(zone == 1)       g_ch1_offset -= LIFT_FINE_STEP;
            else if(zone == -1) g_ch1_offset += LIFT_FINE_STEP;
            g_ch1_zone = zone;
            g_ch1_last_tick = xTaskGetTickCount();
        }
        else
        {
            g_ch1_last_tick = xTaskGetTickCount();
        }
        if(g_ch1_offset >  LIFT_FINE_MAX) g_ch1_offset =  LIFT_FINE_MAX;
        if(g_ch1_offset < -LIFT_FINE_MAX) g_ch1_offset = -LIFT_FINE_MAX;
    }

    switch(g_step)
    {
        case G_IDLE:
            g_flip = FLIP_UP_POS; YV3(0); g_flip_speed = FLIP_UP_SPEED;  break;

        case G_RACK_ZERO:
            g_flip = FLIP_UP_POS; YV3(0); g_flip_speed = FLIP_UP_SPEED;  break;

        case G_CLAW_OPEN:
            if(g_rack <= RACK_ZERO_TOL) YV3(1);   /* 齿条归零才开爪 */
            break;

        case G_FLIP_DOWN:
            g_flip = FLIP_DOWN_POS; g_flip_speed = FLIP_DOWN_SPEED;  break;

        case G_CLAW_CLOSE:
            YV3(0); break;

        case G_FLIP_BACK:
            g_flip = FLIP_UP_POS; g_flip_speed = FLIP_UP_SPEED;  break;

        case G_LIFT_RISE:     /* CH6高度 + CH7翻转 + CH10手动 */
            g_flip = g_flip_toggle ? FLIP_TOGGLE_POS : FLIP_UP_POS;
            g_flip_speed = FLIP_UP_SPEED;
            break;

        default:
            break;
    }
}

/* ================================================================ */

/* 抓取模式下抬升目标: CH6全局三档 */
float grab_lift_target(uint16_t ch6)
{
    if(ch6 > 1300)       return 0.2f;                      /* 下拨→回零 */
    else if(ch6 >= 650)  return GRAB_H_DOCK + g_ch1_offset; /* 中位→对接 */
    else                 return GRAB_H_HIGH + g_ch1_offset; /* 上拨→17 */
}

/* ================================================================ */

/* CH10 全局齿条控制 (无模式限制) */
void grab_update_rack(uint16_t ch10)
{
    g_rack = Map((float)ch10, 326.0f, 1659.0f, 0.0f, 12.0f);
}

/* ================================================================ */

void grab_task(void *parameter)
{
    while(1)
    {
        if(g_sys_enabled)
        {
            float flip = g_flip;
            if(flip > FLIP_MAX) flip = FLIP_MAX;
            if(flip < FLIP_MIN) flip = FLIP_MIN;
            dm_pos_ctrl(&LIFT_CAN, 5, flip, g_flip_speed); vTaskDelay(2);
            dm_pos_ctrl(&LIFT_CAN, 6, g_rack, RACK_SPEED);
        }
        vTaskDelay(20);
    }
}
