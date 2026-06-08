#ifndef _UART_TASK_H_
#define _UART_TASK_H_

#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "sbus_set.h"
#include "CAN_receive.h"

/* Chassis velocity setpoints (defined in main.c) */
extern float set_vx, set_vy, set_vw;

/* Arm position setpoints (defined in uart_task.c) */
extern float fb_des, lr_des, ud_des, end_des;

/* Shared system state (defined in main.c) */
extern bool sys_enabled;
bool ch_high(int ch);

/* ==================== 协议常量 (rc26_vehicle_serial_protocol_final.md) ==================== */

#define CTRL_FRAME_LEN   30   // 控制帧: 上位机→电控
#define STAT_FRAME_LEN   27   // 状态帧: 电控→上位机
#define UART3_SENSOR_FRAME_LEN  8

/* 速度协议: 10000 = 1 m/s, 10000 = 1 rad/s */
#define VEL_SCALE        10000.0f
#define VEL_MAX_MPS      2.0f     // ±2 m/s 限幅
#define VEL_MAX_RADPS    3.2f     // ±3.2 rad/s 限幅

/* DMA receive buffers */
extern uint8_t uart1_rx_buf[CTRL_FRAME_LEN];

/* ==================== 解析后的协议变量 ==================== */

/* 抬升 (控制帧) */
extern uint16_t lift_front_target;   // mm
extern uint16_t lift_back_target;    // mm
extern uint8_t  lift_mode;          // 0=normal, 1=fast

/* 抬升实际高度 (状态帧回传, 来自编码器) */
extern uint16_t lift_front_actual;   // mm
extern uint16_t lift_back_actual;    // mm

/* 机械臂 pitch (控制帧, int16 mrad) */
extern int16_t left_pitch1,  left_pitch2,  left_pitch3;
extern int16_t right_pitch1, right_pitch2, right_pitch3;

/* 末端吸盘 (控制帧, 0=保持 1=吸气 2=松开) */
extern uint8_t left_sucker, right_sucker;

/* 武器头 (控制帧) */
extern uint8_t weapon_pitch;    // 0=保持 1=平行地面 2=垂直地面
extern uint8_t weapon_gripper;  // 0=保持 1=打开 2=闭合

/* 旧兼容变量 (逐步废弃) */
extern float   upstairs_chassis_speed;
extern uint8_t up_dis_front;
extern uint8_t up_dis_back;

/* ==================== API ==================== */

void uart_rx_init(void);
void uart_task(void *parameter);
void uart_tx_ch5_notify(void);

#endif
