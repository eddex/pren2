/*
 * velocity.h
 *
 *  Created on: 08.12.2018
 *      Author: ANDREAS
 */

#ifndef VELOCITY_H_
#define VELOCITY_H_

#define TicksPerRev 48 // Ticks pro Motorumdrehung
#define iGetriebe 9.68 // Untersetzung
#define Wirkdurchmesser 26 // Wirkdurchmesser [mm]
#define Wirkumfang Wirkdurchmesser * 3.141 // Wirkdurchmesser * Pi [mm]
#define Frequency 100 // Aufruffrequenz (10ms = 100Hz)

// Init Routine
void Velo_Init();

// Returns Encoder Velocity in mm/s
int32_t Velo_GetVelo();

// Samples Velocity
void Velo_Sample();

#endif /* VELOCITY_H_ */
