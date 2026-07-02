#ifndef _ARM_H_
#define _ARM_H_

#include "main.h"
#include "sbus_set.h"
#include <stdbool.h>

void arm_init(void);
void arm_enable(void);
void arm_disable(void);

/* select_left=true左臂, tip_45=true→放下时末端朝前 / false→末端朝地 */
void arm_update(SBUS_t *sbus, bool select_left, bool tip_45);

bool arm_is_left_down(void);         /* 左臂是否打下 */
bool arm_is_right_down(void);        /* 右臂是否打下 */
void arm_task(void *parameter);      /* 50Hz CAN发送 */

#endif
