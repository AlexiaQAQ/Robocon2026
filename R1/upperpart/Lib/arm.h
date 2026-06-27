#ifndef _ARM_H_
#define _ARM_H_

#include "main.h"
#include "sbus_set.h"
#include <stdbool.h>

void arm_init(void);
void arm_enable(void);
void arm_disable(void);

/* select_left=true左臂, tip_45=放方块模式(竖起朝前) / false=吸方块(竖起45°兜) */
void arm_update(SBUS_t *sbus, bool select_left, bool tip_45);

void arm_task(void *parameter);      /* 50Hz CAN发送 */

#endif
