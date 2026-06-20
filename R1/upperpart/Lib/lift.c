#include "dm_motor.h"
#include "lift.h"
#include "solenoid_valves.h"
#include "cmsis_os.h"
#include <stdbool.h>

extern bool g_sys_enabled;

#define LIFT_CAN    hcan1
#define LIFT_SPEED  4.0f
#define LIFT_RETURN_SPEED  2.0f

static motor_t  lift_motor[4];
static float    lift_target = 0.2f;      /* 0.2 = 回零 */

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
    while(1)
    {
        if(g_sys_enabled)
        {
            float speed = (lift_target == 0.2f) ? LIFT_RETURN_SPEED : LIFT_SPEED;
            dm_pos_ctrl(&LIFT_CAN, 1, -lift_target, speed); vTaskDelay(2);
            dm_pos_ctrl(&LIFT_CAN, 2,  lift_target, speed); vTaskDelay(2);
            dm_pos_ctrl(&LIFT_CAN, 3, -lift_target, speed); vTaskDelay(2);
            dm_pos_ctrl(&LIFT_CAN, 4,  lift_target, speed); vTaskDelay(2);
            YV_flash(&LIFT_CAN);   /* 电磁阀刷新 */
        }
        vTaskDelay(20);
    }
}
