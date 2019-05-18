/*
 * quad.c
 *
 *  Created on: 08.12.2018
 *      Author: ANDREAS
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "gpio.h"
#include <motor_h.h>
#include <quad_h.h>

int32_t ticks_h;
GPIO_PinState chA_h;
GPIO_PinState chB_h;
int32_t error_h;

enum quad_h_e quad_h;

// Init Routine
void Quad_H_Init(){
	ticks_h = 0;
	// Encoder vorne
	chA_h = HAL_GPIO_ReadPin(GPIOA, Enc_ChA_MOT_H_Pin);
	chB_h = HAL_GPIO_ReadPin(GPIOA, Enc_ChB_MOT_H_Pin);

	// Initialisierung Quadraturencoder
	quad_h = sh00;
	if (chA_h == GPIO_PIN_RESET && chB_h == GPIO_PIN_RESET){
		quad_h = sh00;
	}
	else if (chA_h == GPIO_PIN_RESET && chB_h == GPIO_PIN_SET){
		quad_h = sh01;
	}
	else if (chA_h == GPIO_PIN_SET && chB_h == GPIO_PIN_RESET){
		quad_h = sh10;
	}
	else if (chA_h == GPIO_PIN_SET && chB_h == GPIO_PIN_SET){
		quad_h = sh11;
	}
}

// Returns Encoder Position
int32_t Quad_H_GetPos(){
	return ticks_h;
}

// Samples Encoder
void Quad_H_Sample(){

	//To change the motor direction, change the Enc_ChX_MOT_V_Pin
	chA_h = HAL_GPIO_ReadPin(GPIOA, Enc_ChA_MOT_H_Pin);
	chB_h = HAL_GPIO_ReadPin(GPIOA, Enc_ChB_MOT_H_Pin);

	// sXX: first bit = chA and second bit = chB

	switch (quad_h) {
	case sh00:
		if (chA_h == GPIO_PIN_RESET && chB_h == GPIO_PIN_SET) {
			ticks_h++;
			quad_h = sh01;
		} else if (chA_h == GPIO_PIN_SET && chB_h == GPIO_PIN_RESET) {
			ticks_h--;
			quad_h = sh10;
		} else if (chA_h == GPIO_PIN_SET && chB_h == GPIO_PIN_SET) {
			error_h++;
			quad_h = sh11;
		}
		break;

	case sh01:
		if (chA_h == GPIO_PIN_RESET && chB_h == GPIO_PIN_RESET) {
			ticks_h--;
			quad_h = sh00;
		} else if (chA_h == GPIO_PIN_SET && chB_h == GPIO_PIN_SET) {
			ticks_h++;
			quad_h = sh11;
		} else if (chA_h == GPIO_PIN_SET && chB_h == GPIO_PIN_RESET) {
			error_h++;
			quad_h = sh10;
		}
		break;

	case sh11:
		if (chA_h == GPIO_PIN_RESET && chB_h == GPIO_PIN_SET) {
			ticks_h--;
			quad_h = sh01;
		} else if (chA_h == GPIO_PIN_SET && chB_h == GPIO_PIN_RESET) {
			ticks_h++;
			quad_h = sh10;
		} else if (chA_h == GPIO_PIN_RESET && chB_h == GPIO_PIN_RESET) {
			error_h++;
			quad_h = sh00;
		}
		break;

	case sh10:
		if (chA_h == GPIO_PIN_RESET && chB_h == GPIO_PIN_RESET) {
			ticks_h++;
			quad_h = sh00;
		} else if (chA_h == GPIO_PIN_SET && chB_h == GPIO_PIN_SET) {
			ticks_h--;
			quad_h = sh11;
		} else if (chA_h == GPIO_PIN_RESET && chB_h == GPIO_PIN_SET) {
			error_h++;
			quad_h = sh01;
		}
		break;
	}
}
