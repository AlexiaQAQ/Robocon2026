#ifndef _ARM_H_
#define _ARM_H_

#include "main.h"
#include "sbus_set.h"
#include <stdbool.h>

void arm_init(void);
void arm_enable(void);
void arm_disable(void);

/* CH5中位时调用: select_left=true左臂, ch6_high=3层 */
void arm_update(SBUS_t *sbus, bool select_left, bool ch6_high);

void arm_task(void *parameter);      /* 50Hz CAN发送 */

#endif
