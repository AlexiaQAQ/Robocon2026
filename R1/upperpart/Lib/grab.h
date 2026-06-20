#ifndef _GRAB_H_
#define _GRAB_H_

#include "main.h"
#include "sbus_set.h"
#include <stdbool.h>

void grab_init(void);
void grab_enable(void);
void grab_disable(void);

/* CH5上拨(抓取模式)时调用 */
void grab_update(SBUS_t *sbus);

/* CH5离开抓取模式时调用 → 复位 */
void grab_reset(void);

void grab_task(void *parameter);       /* 50Hz CAN发送 */

#endif
