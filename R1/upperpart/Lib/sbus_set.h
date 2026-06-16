#ifndef _SBUS_SET_H
#define _SBUS_SET_H

#include "main.h"
#include "usart.h"
#include <stdbool.h>

typedef struct
{
	uint16_t ch[16];                    /* 16 通道, 11-bit (0~2047) */
} SBUS_t;

#define sbus_uart huart4

extern unsigned char sbus_rx_buf[25];   /* 接收缓冲 */
extern SBUS_t sbus_ch;                  /* 解析后的通道数据 */
extern volatile bool sbus_frame_ready;  /* ISR 置位, task 清 */

void sbus_rx_init(void);
float Map(float val, float in_min, float in_max, float out_min, float out_max);
int16_t map(int16_t val, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max);

#endif
