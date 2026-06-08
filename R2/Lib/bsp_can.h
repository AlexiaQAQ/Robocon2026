#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void can_filter_init(void);

#endif

