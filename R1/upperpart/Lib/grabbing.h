#ifndef __GRABBING_H_
#define __GRABBING_H_

#include "main.h"
#include "spi.h"
#include "cmsis_os.h"
#include "mcp2515.h"
#include "motor_control.h"
#include "solenoid_valves.h"

#define grabbing_can hcan5

#define GRABBING_STATION_LOWER 0u
#define GRABBING_STATION_UPPER 1u

#define GRABBING_STEP_RETRACT 1u
#define GRABBING_STEP_CLAW_OPEN 2u
#define GRABBING_STEP_FLIP_DOWN 3u
#define GRABBING_STEP_CLAW_CLOSE 4u
#define GRABBING_STEP_FLIP_BACK 5u
#define GRABBING_STEP_PUSH 6u
#define GRABBING_STEP_PUSH_INCREMENT 7u

void grabbing_init(void);
void grabbing_enable(void);
void grabbing_disable(void);

void grabbing_section1(void);
void grabbing_section2(void);
void grabbing_section3(void);
void grabbing_section4(void);
void grabbing_section5(void);
void grabbing_section6(void);
void grabbing_upper_section1(void);
void grabbing_upper_section2(void);
void grabbing_upper_section3(void);
void grabbing_upper_section4(void);
void grabbing_upper_section5(void);
void grabbing_upper_section6(void);
void grabbing_run_step(uint8_t station, uint8_t step);
void grabbing_push_to(uint8_t station, float target);

#endif





