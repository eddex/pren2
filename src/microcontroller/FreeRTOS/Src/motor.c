/*
 * motor.c
 *
 *  Created on: 15.12.2018
 *      Author: ANDREAS
 */

#include "stm32f3xx_hal.h"
#include "tim.h"
#include "motor.h"

uint16_t pwm_value = 0;

// Init Routine
void Motor_Init(){
	pwm_value = 0;
	// Initialisierung PWM
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
}

// Set velocity %
void Motor_SetVelo(int8_t velo){
	if (velo >= 0){
		// IN2 = 0 & IN1 = PWM 	--> Forward
		pwm_value = (velo * PWM_MAX_VALUE)/100;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_value);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm_value);
	} else{
		// IN1 = 0 & IN2 = PWM 	--> Reverse
		pwm_value = (-velo * PWM_MAX_VALUE)/100;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_value);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm_value);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
	}
}

// Break
void Motor_Break(){
	// IN1 = 1 & IN2 = 1 --> Brake
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_MAX_VALUE);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM_MAX_VALUE);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_MAX_VALUE);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_MAX_VALUE);
}
