#ifndef _CAN_RECEIVE_H_
#define _CAN_RECEIVE_H_

#include "motor_control.h"

// CAN 回调函数 — DM 电机反馈接收 & HAL 中断入口
extern void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
extern void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);

#endif
