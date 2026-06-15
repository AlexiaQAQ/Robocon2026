#ifndef _IMU_H
#define _IMU_H

#include "main.h"
#include "cmsis_os.h"
#include "usart.h"

/* HLK-AS201 帧格式 (实际): FA FB ... FC FD, 帧长50字节 */
#define IMU_UART        huart3

/* 欧拉角 (度) */
extern float imu_roll;
extern float imu_pitch;
extern float imu_yaw;
extern float imu_yaw_filtered;
extern uint8_t imu_ok;

void imu_task(void *parameter);

#endif
