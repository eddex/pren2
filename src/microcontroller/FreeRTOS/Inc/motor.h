/*
 * motor.h
 *
 *  Created on: 15.12.2018
 *      Author: ANDREAS
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#define PWM_MAX_VALUE 20000

// Init Routine
void Motor_Init();

// Set velocity
void Motor_SetVelo(int8_t velo);

// Break
void Motor_Break();

#endif /* MOTOR_H_ */
