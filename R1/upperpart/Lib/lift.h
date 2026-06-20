#ifndef _LIFT_H_
#define _LIFT_H_

#include "main.h"

void lift_init(void);
void lift_enable(void);
void lift_disable(void);
void lift_update(float target);          /* 外部设定抬升目标 */
void lift_task(void *parameter);         /* 50Hz CAN 发送 */

#endif
