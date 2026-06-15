#ifndef _SBUS_SET_H
#define _SBUS_SET_H

#include "main.h"
#include "usart.h"

typedef struct 
{
	uint16_t ch[16];                			// 16个字节数据
}SBUS_t;

#define sbus_uart huart4								//	SBUS串口

extern unsigned char sbus_rx_buf[25];		//接收的数据
extern SBUS_t sbus_ch;									//转换的数据

void sbus_rx_init(void);
void rx_set(SBUS_t *sbus);
float Map(float val,float in_min,float in_max,float out_min,float out_max);
int16_t map(int16_t val, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max);

#endif

