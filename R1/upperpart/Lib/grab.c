#include "dm_motor.h"
#include "grab.h"
#include "solenoid_valves.h"
#include "sbus_set.h"
#include "cmsis_os.h"

#define LIFT_CAN    hcan1
#define RACK_SPEED  15.0f          /* 2325齿条 */
#define FLIP_UP_POS      0.0f      /* 4340翻上位置 */
#define FLIP_DOWN_POS   -1.8f      /* 4340翻下取杆位置 */
#define FLIP_TOGGLE_POS -0.4f    /* 4340对接翻转位置 */
#define FLIP_GRAB_POS  -1.0f     /* 4340抓取位置3 */
#define FLIP_MAX         0.0f      /* 4340翻转上限 */
#define FLIP_MIN        -2.0f      /* 4340翻转下限 */
#define FLIP_KP          30.0f     /* 4340 MIT KP */
#define FLIP_KD           5.0f     /* 4340 MIT KD */
#define RACK_ZERO_TOL    0.2f      /* 齿条回零容差, 开爪前检查 */
#define RACK_MAX          15.0f     /* 齿条上限 */
#define RACK_MIN          0.0f      /* 齿条下限 */
#define GRAB_H_HIGH      25.0f     /* 高位 (CH6上拨) */
#define GRAB_H_DOCK      14.2f    /* 对接 (CH6中位) */
#define LIFT_FINE_STEP    0.05f    /* CH1每次微调步长 */
#define LIFT_FINE_MAX     3.0f     /* CH1微调最大累积 */

static motor_t grab_motor[2];       /* [0]=4340翻转 ID5, [1]=2325齿条 ID6 */

/* ---- 工序状态机 ---- */
typedef enum {
    G_IDLE = 0,      /* 空闲 */
    G_RACK_ZERO,     /* 1: 齿条回零 */
    G_CLAW_OPEN,     /* 2: 开夹爪 */
    G_FLIP_DOWN,     /* 3: 翻转取杆 */
    G_CLAW_CLOSE,    /* 4: 关夹爪抓杆 */
    G_FLIP_BACK,     /* 5: 翻回 */
    G_LIFT_RISE,     /* 6: 对接/回零 + CH10手动, CH6切换高度 */
    G_FLIP_GRAB,     /* 7: 翻转→-1.3 */
} grab_step_t;

static grab_step_t g_step = G_IDLE;
static bool     g_last_ch7       = false;  /* CH7 边沿 */
static uint32_t g_grab_last_tick = 0;      /* 上次调用tick, 检测跨模式断档 */
static float g_rack = 0.0f;         /* 2325齿条目标 */
static float g_flip = FLIP_UP_POS;   /* 4340翻转目标 */
static int   g_flip_cycle  = 0;            /* 步骤7 三态循环 0/1/2 */
static float    g_ch1_offset     = 0.0f;  /* CH1摇杆微调偏移 */
static int      g_ch1_zone       = 0;      /* CH1当前区域: -1低/0中/1高 */
static uint32_t g_ch1_last_tick  = 0;      /* 上次调用tick, 检测跨模式断档 */
extern bool g_sys_enabled;

/* ================================================================ */

void grab_init(void)
{
    dm_init(&grab_motor[0], 5, DM_MODE_MIT, DM_4340);
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
    g_rack       = 0.0f;
    g_flip       = FLIP_UP_POS;
    g_flip_cycle = 0;
    g_ch1_offset  = 0.0f;
    YV3(0);
}

/* ================================================================ */

void grab_update(SBUS_t *sbus)
{
    /* CH7 边沿 → 推进工序 */
    uint32_t now = xTaskGetTickCount();
    bool ch15 = (sbus->ch[15] < 650);   /* CH15回弹拨杆 */

    /* 首次调用 或 距上次超过50ms → 仅同步不推进 (防跨模式误触发) */
    if(g_grab_last_tick == 0 || (now - g_grab_last_tick) > pdMS_TO_TICKS(50))
    {
        g_last_ch7 = ch15;
        g_grab_last_tick = now;
    }
    /* 回弹拨杆: 按下+松开=一次边沿, 在松开时触发 */
    else if(ch15 && !g_last_ch7)
    {
        g_last_ch7 = ch15;
        g_grab_last_tick = now;
        if(g_step < G_LIFT_RISE)       g_step++;                  /* 0→5→6 */
        else if(g_step < G_FLIP_GRAB)  { g_step++; g_flip_cycle=0; } /* 6→7 */
        else                           g_flip_cycle = (g_flip_cycle+1)%3; /* 7: 三态循环 */
    }
    else
    {
        g_last_ch7 = ch15;
        g_grab_last_tick = now;
    }

    /* CH10 摇杆增量微调 (全程生效, 边沿触发, 需回中才能再次调节) */
    {
        float ch10 = (float)sbus->ch[10];
        int zone = (ch10 > 1012.0f) ? 1 : (ch10 < 972.0f) ? -1 : 0;

        if(g_ch1_last_tick == 0 || (xTaskGetTickCount() - g_ch1_last_tick) > pdMS_TO_TICKS(50))
        {
            g_ch1_zone = zone;
            g_ch1_last_tick = xTaskGetTickCount();
        }
        else if(zone != g_ch1_zone)
        {
            if(zone == 1)       g_ch1_offset += LIFT_FINE_STEP;
            else if(zone == -1) g_ch1_offset -= LIFT_FINE_STEP;
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
            g_flip = FLIP_UP_POS; YV3(0);  break;

        case G_RACK_ZERO:
            g_flip = FLIP_UP_POS; YV3(0);  break;

        case G_CLAW_OPEN:
            if(g_rack <= RACK_ZERO_TOL) YV3(1);   /* 齿条归零才开爪 */
            break;

        case G_FLIP_DOWN:
            g_flip = FLIP_DOWN_POS;  break;

        case G_CLAW_CLOSE:
            YV3(0); break;

        case G_FLIP_BACK:
            g_flip = FLIP_UP_POS;  break;

        case G_LIFT_RISE:     /* CH6高度 + CH10手动 */
            g_flip = FLIP_UP_POS;
            break;

        case G_FLIP_GRAB:     /* 三态循环: 0→-0.4→-1.0 */
            g_flip = (g_flip_cycle == 0) ? FLIP_UP_POS :
                     (g_flip_cycle == 1) ? FLIP_TOGGLE_POS : FLIP_GRAB_POS;
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

/* CH10 增量齿条: 前推→伸出, 后拉→缩回 (仅设方向, 实际步进在grab_task) */
void grab_update_rack(uint16_t ch8)
{
    g_rack = Map((float)ch8, 326.0f, 1659.0f, RACK_MIN, RACK_MAX);
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
            dm_mit_ctrl(&LIFT_CAN, &grab_motor[0], flip, 0.0f, FLIP_KP, FLIP_KD, 0.0f);
            vTaskDelay(2);

            /* 齿条: 硬限幅 */
            if(g_rack > RACK_MAX) g_rack = RACK_MAX;
            if(g_rack < RACK_MIN) g_rack = RACK_MIN;
            dm_pos_ctrl(&LIFT_CAN, 6, g_rack, RACK_SPEED);
        }
        vTaskDelay(20);
    }
}
