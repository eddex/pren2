/*
 * velocity.h
 *
 *  Created on: 08.12.2018
 *      Author: ANDREAS
 */

#ifndef VELOCITY_V_H_
#define VELOCITY_V_H_

#include <stdint.h>

// Init Routine
void Velo_V_Init();

// Returns Encoder Velocity in ticks/s
int32_t Velo_V_GetVelo();

// Samples Velocity
void Velo_V_Sample();

#endif /* VELOCITY_V_H_ */
