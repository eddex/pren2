/*
 * velocity.c
 *
 *  Created on: 08.12.2018
 *      Author: ANDREAS
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "velocity.h"
#include "quad.h"
#include "motor.h"

int32_t velocity;
int32_t oldPos;

// Init Routine
void Velo_Init(){
	velocity = 0;
	oldPos = 0;
}

// Returns Encoder Velocity in mm/s
int32_t Velo_GetVelo(){
	return velocity;
}

int32_t log_velo[100];
int16_t VelCounter = 0;
// Samples Velocity
void Velo_Sample(){
	int32_t newPos = Quad_GetPos();
	int32_t diffDist = newPos - oldPos; // Differenzticks zwischen alter und neuer Position
	velocity = diffDist * Frequency;
	oldPos = newPos;

	log_velo[VelCounter] = velocity;
	VelCounter++;
	if (VelCounter >= 100){
		VelCounter = 0;
	}
}
