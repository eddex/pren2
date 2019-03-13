/*
 * servo.h
 *
 *  Created on: 08.03.2019
 *      Author: ANDREAS
 */

#ifndef SERVO_H_
#define SERVO_H_

// Init method
void Servo_Init();

// Setter method for servo angle 0..90°
void Servo_SetAngle(uint8_t angle);

// Getter method for servo angle 0..90°
uint8_t Servo_GetAngle();

#endif /* SERVO_H_ */
