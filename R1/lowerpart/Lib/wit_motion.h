#ifndef _WIT_MOTION_H_
#define _WIT_MOTION_H_

#include "main.h"
#include "usart.h"
#include <stdbool.h>

#define WIT_UART  huart8       /* USART6: PG9(RX) / PG14(TX), 115200-8N1 */
#define WIT_PKT_LEN  11        /* WT901C 角度包 11 字节 */

/* 欧拉角 (°) */
extern float wit_roll;
extern float wit_pitch;
extern float wit_yaw;
extern volatile bool wit_updated;   /* ISR 置位, 新数据到达 */

void wit_init(void);

#endif
