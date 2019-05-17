/*
 * velocity.c
 *
 *  Created on: 08.12.2018
 *      Author: ANDREAS
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include <motor_h.h>
#include <quad_h.h>
#include <velocity_h.h>

int32_t velocity_h;
int32_t oldPos_h;

// Init Routine
void Velo_H_Init(){
	velocity_h = 0;
	oldPos_h = 0;
}

// Returns Encoder Velocity in ticks/s
int32_t Velo_H_GetVelo(){
	return velocity_h;
}

// Samples Velocity
void Velo_H_Sample(){
	int32_t newPos_h = Quad_H_GetPos();
	int32_t diffDist_h = newPos_h - oldPos_h; // Differenzticks zwischen alter und neuer Position
	velocity_h = diffDist_h * Frequency;
	oldPos_h = newPos_h;
}
