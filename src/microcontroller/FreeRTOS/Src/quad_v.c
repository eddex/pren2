/*
 * quad.c
 *
 *  Created on: 08.12.2018
 *      Author: ANDREAS
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "gpio.h"
#include <motor_v.h>
#include <quad_v.h>

int32_t ticks_v;
GPIO_PinState chA_v;
GPIO_PinState chB_v;
int32_t error_v;

enum quad_v_e quad_v;

// Init Routine
void Quad_V_Init(){
	ticks_v = 0;
	// Encoder vorne
	chA_v = HAL_GPIO_ReadPin(GPIOA, Enc_ChB_MOT_V_Pin);
	chB_v = HAL_GPIO_ReadPin(GPIOA, Enc_ChA_MOT_V_Pin);

	// Initialisierung Quadraturencoder
	quad_v = sv00;
	if (chA_v == GPIO_PIN_RESET && chB_v == GPIO_PIN_RESET){
		quad_v = sv00;
	}
	else if (chA_v == GPIO_PIN_RESET && chB_v == GPIO_PIN_SET){
		quad_v = sv01;
	}
	else if (chA_v == GPIO_PIN_SET && chB_v == GPIO_PIN_RESET){
		quad_v = sv10;
	}
	else if (chA_v == GPIO_PIN_SET && chB_v == GPIO_PIN_SET){
		quad_v = sv11;
	}
}

// Returns Encoder Position
int32_t Quad_V_GetPos(){
	return ticks_v;
}

// Samples Encoder
void Quad_V_Sample(){

	//To change the motor direction, change the Enc_ChX_MOT_V_Pin
	chA_v = HAL_GPIO_ReadPin(GPIOA, Enc_ChB_MOT_V_Pin);
	chB_v = HAL_GPIO_ReadPin(GPIOA, Enc_ChA_MOT_V_Pin);

	// sXX: first bit = chA and second bit = chB

	switch (quad_v) {
	case sv00:
		if (chA_v == GPIO_PIN_RESET && chB_v == GPIO_PIN_SET) {
			ticks_v++;
			quad_v = sv01;
		} else if (chA_v == GPIO_PIN_SET && chB_v == GPIO_PIN_RESET) {
			ticks_v--;
			quad_v = sv10;
		} else if (chA_v == GPIO_PIN_SET && chB_v == GPIO_PIN_SET) {
			error_v++;
			quad_v = sv11;
		}
		break;

	case sv01:
		if (chA_v == GPIO_PIN_RESET && chB_v == GPIO_PIN_RESET) {
			ticks_v--;
			quad_v = sv00;
		} else if (chA_v == GPIO_PIN_SET && chB_v == GPIO_PIN_SET) {
			ticks_v++;
			quad_v = sv11;
		} else if (chA_v == GPIO_PIN_SET && chB_v == GPIO_PIN_RESET) {
			error_v++;
			quad_v = sv10;
		}
		break;

	case sv11:
		if (chA_v == GPIO_PIN_RESET && chB_v == GPIO_PIN_SET) {
			ticks_v--;
			quad_v = sv01;
		} else if (chA_v == GPIO_PIN_SET && chB_v == GPIO_PIN_RESET) {
			ticks_v++;
			quad_v = sv10;
		} else if (chA_v == GPIO_PIN_RESET && chB_v == GPIO_PIN_RESET) {
			error_v++;
			quad_v = sv00;
		}
		break;

	case sv10:
		if (chA_v == GPIO_PIN_RESET && chB_v == GPIO_PIN_RESET) {
			ticks_v++;
			quad_v = sv00;
		} else if (chA_v == GPIO_PIN_SET && chB_v == GPIO_PIN_SET) {
			ticks_v--;
			quad_v = sv11;
		} else if (chA_v == GPIO_PIN_RESET && chB_v == GPIO_PIN_SET) {
			error_v++;
			quad_v = sv01;
		}
		break;
	}
}
