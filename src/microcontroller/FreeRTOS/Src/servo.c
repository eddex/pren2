/*
 * servo.c
 *
 *  Created on: 08.03.2019
 *      Author: ANDREAS
 */

#include "stm32f3xx_hal.h"
#include "tim.h"
#include "servo.h"

/* Servo Angle-Control:
* 0°	=> 1.1ms (PWM_Value = T0° / (1/f)) = 26'400
* 90°	=> 1.9ms (PWM_Value = T90° / (1/f)) = 45'600
*/

#define tim2Freq 24000000.0 // Timer 2 Frequenz [Hz]
#define tim2Period (1000000.0/tim2Freq)// Timer 2 Periode [us]
#define period_0 1100 // Periode für 0° [us]
#define period_90 1900 // Periode für 90° [us]
#define pwmValue_0 (period_0/tim2Period) // PWM value for 0°
#define pwmValue_90 (period_90/tim2Period) // PWM value for 90°

uint32_t servoPWM = 0;
uint8_t oldAngle = 0;

// Init method
void Servo_Init(){
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint16_t)pwmValue_0); // 0° anfahren
}

// Setter method for servo angle 0..90°
void Servo_SetAngle(uint8_t angle){
	if (angle > 90){
		angle = 90;
	}
	oldAngle = angle;
	servoPWM = (((pwmValue_90-pwmValue_0)*angle)/90)+pwmValue_0;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint16_t) servoPWM);
}

// Getter method for servo angle 0..90°
uint8_t Servo_GetAngle(){
	return oldAngle;
}

