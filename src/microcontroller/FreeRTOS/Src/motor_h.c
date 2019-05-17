/*
 * motor.c
 *
 *  Created on: 15.12.2018
 *      Author: ANDREAS
 */

#include "stm32f3xx_hal.h"
#include "tim.h"
#include "motor_h.h"

uint16_t pwm_value_h = 0;

// Init Routine
void Motor_H_Init(){
	pwm_value_h = 0;
	// Initialisierung PWM
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
}

// Set velocity %
void Motor_H_SetVelo(int8_t velo){
	if (velo >= 0){
		// IN2 = 0 & IN1 = PWM 	--> Forward
		pwm_value_h = (velo * PWM_MAX_VALUE)/100;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm_value_h);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
	} else{
		// IN1 = 0 & IN2 = PWM 	--> Reverse
		pwm_value_h = (-velo * PWM_MAX_VALUE)/100;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm_value_h);
	}
}

// Break
void Motor_H_Break(){
	// IN1 = 1 & IN2 = 1 --> Brake
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_MAX_VALUE);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_MAX_VALUE);
}
