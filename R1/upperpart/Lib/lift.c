#include "dm_motor.h"
#include "lift.h"
#include "solenoid_valves.h"
#include "cmsis_os.h"
#include <stdbool.h>

extern bool g_sys_enabled;

#define LIFT_CAN    hcan1
#define LIFT_SPEED  4.5f
#define LIFT_RETURN_TARGET 0.2f    /* 回零高度 */
#define LIFT_MAX    29.0f          /* 抬升硬限幅上限 */
#define LIFT_MIN     0.2f          /* 抬升硬限幅下限 */
#define LIFT_SLEW    1.0f          /* 每20ms最大步进, 防切换抽搐 */

static motor_t  lift_motor[4];
static float    lift_target = LIFT_RETURN_TARGET;

void lift_init(void)
{
    dm_init(&lift_motor[0], 1, DM_MODE_POS, DM_4340);
    dm_init(&lift_motor[1], 2, DM_MODE_POS, DM_4340);
    dm_init(&lift_motor[2], 3, DM_MODE_POS, DM_4340);
    dm_init(&lift_motor[3], 4, DM_MODE_POS, DM_4340);
}

void lift_enable(void)
{
    dm_enable(&LIFT_CAN, &lift_motor[0]); vTaskDelay(2);
    dm_enable(&LIFT_CAN, &lift_motor[1]); vTaskDelay(2);
    dm_enable(&LIFT_CAN, &lift_motor[2]); vTaskDelay(2);
    dm_enable(&LIFT_CAN, &lift_motor[3]); vTaskDelay(2);
}

void lift_disable(void)
{
    dm_disable(&LIFT_CAN, &lift_motor[0]); vTaskDelay(2);
    dm_disable(&LIFT_CAN, &lift_motor[1]); vTaskDelay(2);
    dm_disable(&LIFT_CAN, &lift_motor[2]); vTaskDelay(2);
    dm_disable(&LIFT_CAN, &lift_motor[3]); vTaskDelay(2);
}

void lift_update(float target)
{
    lift_target = target;
}

void lift_task(void *parameter)
{
    static float g_lift_cmd = LIFT_RETURN_TARGET;  /* 斜坡平滑后的实际指令 */

    while(1)
    {
        if(g_sys_enabled)
        {
            /* 硬限幅 */
            float target = lift_target;
            if(target > LIFT_MAX) target = LIFT_MAX;
            if(target < LIFT_MIN) target = LIFT_MIN;

            /* 斜率限制: 逐步逼近目标, 防模式切换抽搐 */
            if(target > g_lift_cmd + LIFT_SLEW)
                g_lift_cmd += LIFT_SLEW;
            else if(target < g_lift_cmd - LIFT_SLEW)
                g_lift_cmd -= LIFT_SLEW;
            else
                g_lift_cmd = target;

            float speed = LIFT_SPEED;
            dm_pos_ctrl(&LIFT_CAN, 1, -g_lift_cmd, speed); vTaskDelay(2);
            dm_pos_ctrl(&LIFT_CAN, 2,  g_lift_cmd, speed); vTaskDelay(2);
            dm_pos_ctrl(&LIFT_CAN, 3, -g_lift_cmd, speed); vTaskDelay(2);
            dm_pos_ctrl(&LIFT_CAN, 4,  g_lift_cmd, speed); vTaskDelay(2);
            YV_flash(&LIFT_CAN);   /* 电磁阀刷新 */
        }
        vTaskDelay(20);
    }
}
