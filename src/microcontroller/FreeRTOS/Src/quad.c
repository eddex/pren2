/*
 * quad.c
 *
 *  Created on: 08.12.2018
 *      Author: ANDREAS
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "quad.h"
#include "gpio.h"

int32_t position;
GPIO_PinState chA;
GPIO_PinState chB;
int32_t error;

enum quad_e quad;

// Init Routine
void Quad_Init(){
	position = 0;
	chA = HAL_GPIO_ReadPin(GPIOA, Enc_ChA_MOT_H_Pin);
	chB = HAL_GPIO_ReadPin(GPIOA, Enc_ChB_MOT_H_Pin);

	// Initialisierung Quadraturencoder
	quad = s00;
	if (chA == GPIO_PIN_RESET && chB == GPIO_PIN_RESET){
		quad = s00;
	}
	else if (chA == GPIO_PIN_RESET && chB == GPIO_PIN_SET){
		quad = s01;
	}
	else if (chA == GPIO_PIN_SET && chB == GPIO_PIN_RESET){
		quad = s10;
	}
	else if (chA == GPIO_PIN_SET && chB == GPIO_PIN_SET){
		quad = s11;
	}
}

// Returns Encoder Position
int32_t Quad_GetPos(){
	return position;
}

// Samples Encoder
void Quad_Sample(){
	chA = HAL_GPIO_ReadPin(GPIOA, Enc_ChA_MOT_H_Pin);
	chB = HAL_GPIO_ReadPin(GPIOA, Enc_ChB_MOT_H_Pin);

	// sXX: first bit = chA and second bit = chB

	switch (quad) {
	case s00:
		if (chA == GPIO_PIN_RESET && chB == GPIO_PIN_SET) {
			position++;
			quad = s01;
		} else if (chA == GPIO_PIN_SET && chB == GPIO_PIN_RESET) {
			position--;
			quad = s10;
		} else if (chA == GPIO_PIN_SET && chB == GPIO_PIN_SET) {
			error++;
			quad = s11;
		}
		break;

	case s01:
		if (chA == GPIO_PIN_RESET && chB == GPIO_PIN_RESET) {
			position--;
			quad = s00;
		} else if (chA == GPIO_PIN_SET && chB == GPIO_PIN_SET) {
			position++;
			quad = s11;
		} else if (chA == GPIO_PIN_SET && chB == GPIO_PIN_RESET) {
			error++;
			quad = s10;
		}
		break;

	case s11:
		if (chA == GPIO_PIN_RESET && chB == GPIO_PIN_SET) {
			position--;
			quad = s01;
		} else if (chA == GPIO_PIN_SET && chB == GPIO_PIN_RESET) {
			position++;
			quad = s10;
		} else if (chA == GPIO_PIN_RESET && chB == GPIO_PIN_RESET) {
			error++;
			quad = s00;
		}
		break;

	case s10:
		if (chA == GPIO_PIN_RESET && chB == GPIO_PIN_RESET) {
			position++;
			quad = s00;
		} else if (chA == GPIO_PIN_SET && chB == GPIO_PIN_SET) {
			position--;
			quad = s11;
		} else if (chA == GPIO_PIN_RESET && chB == GPIO_PIN_SET) {
			error++;
			quad = s01;
		}
		break;
	}
}