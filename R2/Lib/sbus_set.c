#include "sbus_set.h"

unsigned char sbus_rx_buf[25]={0};

SBUS_t sbus_ch;

void sbus_rx_init()
{
	HAL_UART_Receive_DMA(&sbus_uart,sbus_rx_buf,25);
}

void rx_set(SBUS_t *sbus)
{
	uint8_t cnt = 0;
	
	if(sbus_rx_buf[23] == 0x00 && sbus_rx_buf[0] == 0x0f && sbus_rx_buf[24] == 0x00)
	{
		sbus->ch[0] =((sbus_rx_buf[2]<<8)   + (sbus_rx_buf[1])) & 0x07ff;
		sbus->ch[1] =((sbus_rx_buf[3]<<5)   + (sbus_rx_buf[2]>>3)) & 0x07ff;
		sbus->ch[2] =((sbus_rx_buf[5]<<10)  + (sbus_rx_buf[4]<<2) + (sbus_rx_buf[3]>>6)) & 0x07ff;
		sbus->ch[3] =((sbus_rx_buf[6]<<7)   + (sbus_rx_buf[5]>>1)) & 0x07ff;
		sbus->ch[4] =((sbus_rx_buf[7]<<4)   + (sbus_rx_buf[6]>>4)) & 0x07ff;
		sbus->ch[5] =((sbus_rx_buf[9]<<9)   + (sbus_rx_buf[8]<<1) + (sbus_rx_buf[7]>>7)) & 0x07ff;
		sbus->ch[6] =((sbus_rx_buf[10]<<6)  + (sbus_rx_buf[9]>>2)) & 0x07ff;
		sbus->ch[7] =((sbus_rx_buf[11]<<3)  + (sbus_rx_buf[10]>>5)) & 0x07ff;
		sbus->ch[8] =((sbus_rx_buf[13]<<8)  + (sbus_rx_buf[12])) & 0x07ff;
		sbus->ch[9] =((sbus_rx_buf[14]<<5)  + (sbus_rx_buf[13]>>3)) & 0x07ff;
		sbus->ch[10]=((sbus_rx_buf[16]<<10) + (sbus_rx_buf[15]<<2) + (sbus_rx_buf[14]>>6)) & 0x07ff;
		sbus->ch[11]=((sbus_rx_buf[17]<<7)  + (sbus_rx_buf[16]>>1)) & 0x07ff;
		sbus->ch[12]=((sbus_rx_buf[18]<<4)  + (sbus_rx_buf[17]>>4)) & 0x07ff;
		sbus->ch[13]=((sbus_rx_buf[20]<<9)  + (sbus_rx_buf[19]<<1) + (sbus_rx_buf[18]>>7)) & 0x07ff;
		sbus->ch[14]=((sbus_rx_buf[21]<<6)  + (sbus_rx_buf[20]>>2)) & 0x07ff;
		sbus->ch[15]=((sbus_rx_buf[22]<<3)  + (sbus_rx_buf[21]>>5)) & 0x07ff;
	}
	else
	{
		HAL_UART_DMAStop(&sbus_uart);
		for(cnt=0;cnt<25;cnt++)
		{
			sbus_rx_buf[cnt]=0;
		}
		for(cnt=0;cnt<25;cnt++)
		{
			sbus->ch[cnt]=0;
		}
		HAL_UART_Receive_DMA(&sbus_uart,sbus_rx_buf,25);//重新打开接收
	}
}

float Map(float val,float in_min,float in_max,float out_min,float out_max)
{
	float rec = (float)(val-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
	if(rec>out_max){rec = out_max;}
	if(rec<out_min){rec = out_min;}
	return rec;
}

int16_t map(int16_t val, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max)
{
    if (in_min == in_max) return out_min;

    int32_t result = ((int32_t)(val - in_min) * (out_max - out_min)) / (in_max - in_min) + out_min;

    if (result > out_max) result = out_max;
    if (result < out_min) result = out_min;

    if (result > INT16_MAX) result = INT16_MAX;
    if (result < INT16_MIN) result = INT16_MIN;

    return (int16_t)result;
}


