/*
 * pid.c
 *
 *  Created on: 14.12.2018
 *      Author: ANDREAS
 */

#include "stm32f3xx_hal.h"
#include "pid.h"
#include "quad.h"
#include "velocity.h"
#include "motor.h"

uint8_t pidSetEnable = 0;

int32_t meas_velo;
int32_t integral_v;
int32_t meas_pos;
int32_t integral_p;
int32_t error;
int32_t pVal;
int32_t iVal;
int32_t pidVal;

// Init Routine
void PID_Init(){
	meas_velo = 0;
	integral_v = 0;
	meas_pos = 0;
	integral_p = 0;
	error = 0;
	pVal = 0;
	iVal = 0;
	pidVal = 0;
}


// Geschwindigkeitsregler U/s
void PID_Velo(int32_t set_velo){
	meas_velo = Velo_GetVelo();
	error = set_velo - meas_velo;
	pVal = (Kp_v * error) / 1000;
	integral_v += error;
	iVal = (Ki_v * integral_v) / 1000;
	pidVal = pVal + iVal;
	if (pidVal > 100) {
		Motor_SetVelo(100);
	} else if (pidVal < -100) {
		Motor_SetVelo(-100);
	} else {
		Motor_SetVelo(pidVal);
	}
}

// Positionsregler
void PID_Pos(int32_t set_pos){
	meas_pos = Quad_GetPos();
	error = set_pos - meas_pos;
	pVal = (Kp_p * error) / 1000;
	integral_p += error;
	// Anti-Windup
	if(integral_p >= Aw_p){
		integral_p = Aw_p;
	} else if(integral_p <= -Aw_p){
		integral_p = -Aw_p;
	}
	// Falls Position erreicht Integral zurücksetzen
	if (error == 0) {
		integral_p = 0;
	}
	iVal = (Ki_p * integral_p) / 1000;
	pidVal = pVal + iVal;
	if (pidVal > MAX_VELO) {
		Motor_SetVelo(MAX_VELO);
	} else if (pidVal < -MAX_VELO) {
		Motor_SetVelo(-MAX_VELO);
	} else {
		Motor_SetVelo(pidVal);
	}
}

void PID_ClearError(){
	iVal = 0;
	pVal = 0;
}

void PID_SetEnable(uint8_t enableFlag){
	pidSetEnable = enableFlag;
}

uint8_t PID_GetEnable(void){
	return pidSetEnable;
}
