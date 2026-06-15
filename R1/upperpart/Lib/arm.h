#ifndef __ARM_H_
#define __ARM_H_

#include "main.h"
#include <math.h>
#include <stdbool.h>
#include "can.h"
#include "motor_control.h"

extern float fb_des, lr_des, ud_des, end_des;

void arm_enable(void);
void arm_disable(void);
void arm_ctrl(float terminal,float elbow,float pitch,float yaw);
bool drive_arm_to_ik_2d(float x, float y, float phi);
bool drive_arm_to_ik_3d(float x, float y, float z, float phi);

void arm_back_zero(void);
void arm_ik_test(void);

// CAN 句柄宏 — 更换总线只需改这里
#define ARM_CAN   hcan1

/*
 *  机械臂结构与关节方向
 *
 *    //[关节0x04]---♦ 末端(terminal)
 *    //    ↑              ↑ terminal(+)=末端翘起
 *    //    |              ↓ terminal(-)=末端下垂
 *    //[关节0x03]  ← elbow(+)=向上折, elbow(-)=向下展
 *    //    |
 *    //    |
 *    //[关节0x02]  ← pitch(+)=向前倾, pitch(-)=向后仰
 *    //    |
 *    //    |
 *    //[关节0x01]  ← yaw(+)=俯视逆时针, yaw(-)=俯视顺时针
 *    //    |
 *    //    ● 根部
 *
 *  CAN ID: 0x01=yaw  0x02=pitch  0x03=elbow  0x04=terminal
 *
 *  零位 (全=0): 上臂水平前伸, 前臂垂直向下, 末端朝下
 *
 *  限幅 (rad):  yaw=[-3.14, +1.0]  pitch=[-0.7, +0.7]
 *              elbow=[-3.14, +0.7]  terminal=[-1.57, +1.57]
 *
 *  尺寸 (mm): L1=40  L2=345  L3=370  L4=50
 */

#define L1 40.0f
#define L2 345.0f
#define L3 370.0f
#define L4 50.0f

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define DEG2RAD (3.14159f / 180.0f)

#endif
