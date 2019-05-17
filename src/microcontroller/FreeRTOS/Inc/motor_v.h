/*
 * motor.h
 *
 *  Created on: 15.12.2018
 *      Author: ANDREAS
 */

#ifndef MOTOR_V_H_
#define MOTOR_V_H_

#include <stdint.h>

#define TicksPerRev 48 								// Ticks pro Motorumdrehung
#define iGetriebe 9.68 								// Untersetzung
#define Wirkdurchmesser 26 							// Wirkdurchmesser [mm]
#define Wirkumfang Wirkdurchmesser * 3.141 			// Wirkdurchmesser * Pi [mm]
#define Frequency 100 								// Aufruffrequenz (10ms = 100Hz)

#define PWM_MAX_VALUE 5000

// Init Routine
void Motor_V_Init();

// Set velocity
void Motor_V_SetVelo(int8_t velo);

// Break
void Motor_V_Break();

#endif /* MOTOR_V_H_ */
