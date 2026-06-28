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

void arm_task(void *parameter);      /* 50Hz CAN发送 */

#endif
