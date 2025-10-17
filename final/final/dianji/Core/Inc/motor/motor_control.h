#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "main.h"

/* ------------------ 运动参数定义 ------------------ */
#define MAX_SPEED 400
#define SPEED_STEP 150
#define SPEED_UPDATE_INTERVAL 30
#define TURN_DURATION 800
#define TURN_SPEED 300
#define DIR_STOP 0
#define DIR_FORWARD 1
#define DIR_BACKWARD 2
#define DIR_LEFT 3
#define DIR_RIGHT 4
#define DIR_TURNING 5

void ControlMotor(uint8_t direction, uint32_t speed);
void UpdateSpeedRamp(void);

#endif /* __MOTOR_CONTROL_H */