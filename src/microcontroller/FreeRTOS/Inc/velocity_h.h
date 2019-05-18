/*
 * velocity.h
 *
 *  Created on: 08.12.2018
 *      Author: ANDREAS
 */

#ifndef VELOCITY_H_H_
#define VELOCITY_H_H_

#include <stdint.h>

// Init Routine
void Velo_H_Init();

// Returns Encoder Velocity in ticks/s
int32_t Velo_H_GetVelo();

// Samples Velocity
void Velo_H_Sample();

#endif /* VELOCITY_V_H_ */
