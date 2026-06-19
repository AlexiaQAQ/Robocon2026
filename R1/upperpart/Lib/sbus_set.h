#ifndef __SBUS_SET_H
#define __SBUS_SET_H

#include "main.h"
#include "usart.h"
#include <stdbool.h>

#define SBUS_FRAME_LEN      25
#define SBUS_DMA_BUF_LEN    64
#define SBUS_CH_NUM         16

#define sbus_uart huart4

typedef struct
{
    uint16_t ch[16];

    uint8_t ch17;
    uint8_t ch18;

    uint8_t frame_lost;
    uint8_t failsafe;

}SBUS_t;

extern volatile bool sbus_frame_ready;
extern SBUS_t sbus_ch;

void sbus_rx_init(void);
void sbus_poll(void);

float Map(float val,
          float in_min,
          float in_max,
          float out_min,
          float out_max);

int16_t map(int16_t val,
            int16_t in_min,
            int16_t in_max,
            int16_t out_min,
            int16_t out_max);

#endif
		  