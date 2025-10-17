#include "motor_control.h"
#include "tim.h"

extern uint8_t target_direction;
extern uint8_t current_direction;
extern uint32_t current_speed;
extern uint32_t target_speed;
extern uint32_t last_speed_update_time;
extern uint32_t turn_start_time;
extern uint8_t turn_direction;
extern uint8_t is_turning;
extern uint8_t turn_state;

/* 控制电机 */
void ControlMotor(uint8_t direction, uint32_t speed)
{
    switch(direction)
    {
        case DIR_STOP:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
            break;
        case DIR_FORWARD:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)(speed*0.98));
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
            break;
        case DIR_BACKWARD:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, speed);
            break;
        case DIR_LEFT:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, TURN_SPEED);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
            break;
        case DIR_RIGHT:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, TURN_SPEED);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
            break;
        case DIR_TURNING:
            if (turn_direction == DIR_LEFT) {
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, TURN_SPEED);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
            }
            else {
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, TURN_SPEED);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
            }
            break;
    }
}

/* 速度斜坡 */
void UpdateSpeedRamp(void)
{
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_speed_update_time >= SPEED_UPDATE_INTERVAL)
    {
        last_speed_update_time = current_time;
        
        if (target_direction == DIR_STOP && current_direction != DIR_STOP)
        {
            current_speed = 0;
            current_direction = DIR_STOP;
            ControlMotor(DIR_STOP, 0);
            return; 
        }
        if (is_turning && turn_state == 1)
        {
            if (current_time - turn_start_time >= TURN_DURATION) {
                turn_state = 2;
                is_turning = 0;
                current_direction = DIR_STOP;
                target_speed = 0;
                ControlMotor(DIR_STOP, 0);
            } else {
                ControlMotor(DIR_TURNING, 0);
                return;
            }
        }
        if (target_direction != current_direction)
        {
            target_speed = 0;
            if (current_speed > 0)
            {
                current_speed -= SPEED_STEP;
                ControlMotor(current_direction, current_speed);
            }
            else
            {
                current_direction = target_direction;
                if (current_direction == DIR_LEFT || current_direction == DIR_RIGHT) {
                    is_turning = 1;
                    turn_state = 1;
                    turn_direction = current_direction;
                    turn_start_time = current_time;
                    ControlMotor(DIR_TURNING, 0);
                } else {
                    if (current_direction != DIR_STOP)
                        target_speed = MAX_SPEED;
                }
            }
        }
        else if (current_direction != DIR_STOP && !is_turning)
        {
            if (current_speed < target_speed)
                current_speed += SPEED_STEP;
            else if (current_speed > target_speed)
                current_speed -= SPEED_STEP;
            ControlMotor(current_direction, current_speed);
        }
    }
}