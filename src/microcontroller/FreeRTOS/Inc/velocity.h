/*
 * velocity.h
 *
 *  Created on: 08.12.2018
 *      Author: ANDREAS
 */

#ifndef VELOCITY_H_
#define VELOCITY_H_

// Init Routine
void Velo_Init();

// Returns Encoder Velocity in U/s
int32_t Velo_GetVelo();

// Samples Velocity
void Velo_Sample();

#endif /* VELOCITY_H_ */
