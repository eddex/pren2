/*
 * velocity.c
 *
 *  Created on: 08.12.2018
 *      Author: ANDREAS
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include <motor_v.h>
#include <quad_v.h>
#include <velocity_v.h>

int32_t velocity_v;
int32_t oldPos_v;

// Init Routine
void Velo_V_Init(){
	velocity_v = 0;
	oldPos_v = 0;
}

// Returns Encoder Velocity in ticks/s
int32_t Velo_V_GetVelo(){
	return velocity_v;
}

// Samples Velocity
void Velo_V_Sample(){
	int32_t newPos_v = Quad_V_GetPos();
	int32_t diffDist_v = newPos_v - oldPos_v; // Differenzticks zwischen alter und neuer Position
	velocity_v = diffDist_v * Frequency;
	oldPos_v = newPos_v;
}
