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

int32_t set_velo_ticks = 0;
int32_t meas_velo_ticks = 0;
int32_t integral_v = 0;
int32_t set_pos_ticks = 0;
int32_t meas_pos_ticks = 0;
int32_t integral_p = 0;
int32_t error = 0;
int32_t pVal = 0;
int32_t iVal = 0;
int32_t pidVal = 0;

// Init Routine
void PID_Init(){
	pidSetEnable = 0;
	set_velo_ticks = 0;
	meas_velo_ticks = 0;
	integral_v = 0;
	set_pos_ticks = 0;
	meas_pos_ticks = 0;
	integral_p = 0;
	error = 0;
	pVal = 0;
	iVal = 0;
	pidVal = 0;
}

// Geschwindigkeitsregler mm/s
void PID_Velo(int32_t set_velo){
	set_velo_ticks =  (set_velo * iGetriebe * TicksPerRev) / (Wirkumfang);

	meas_velo_ticks = Velo_GetVelo();
	error = set_velo_ticks - meas_velo_ticks;
	pVal = (Kp_v * error) / 1000;

	// Falls Geschwindigkeit 0 sein soll und 0 ist
	if(set_velo_ticks == 0 && meas_velo_ticks == 0){
		integral_v = 0;
	}

	integral_v += error;
	/*
	// Anti Reset Windup
	if(integral_v >= Aw_v){
		integral_v = Aw_v;
	} else if(integral_v <= -Aw_v){
		integral_v = -Aw_v;
	}
	*/
	iVal = (Ki_v * integral_v) / 1000;
	pidVal = pVal + iVal;
	// Begrenzung falls PWM-Wert grösser 100% wird
	if (pidVal > 100) {
		integral_v -= error; // Anti Reset Windup
		Motor_SetVelo(100);
	} else if (pidVal < -100) {
		integral_v -= error; // Anti Reset Windup
		Motor_SetVelo(-100);
	} else {
		Motor_SetVelo(pidVal);
	}
}

// Positionsregler mm
void PID_Pos(int32_t set_pos){
	set_pos_ticks = (set_pos * iGetriebe * TicksPerRev) / (Wirkumfang);

	meas_pos_ticks = Quad_GetPos();
	error = set_pos_ticks - meas_pos_ticks;
	pVal = (Kp_p * error) / 1000;
	integral_p += error;
	// Anti Reset Windup
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

// Position erreicht
// 0 = Position nicht erreicht
// 1 = Position erreicht
uint8_t PID_InPos(){
	int32_t diff = set_pos_ticks-meas_pos_ticks;
	if (diff < 0){
		diff=diff*(-1);
	}
	return (diff < IN_POS_RANGE);
}

void PID_ClearError(){
	integral_v = 0;
	integral_p = 0;
}

void PID_SetEnable(uint8_t enableFlag){
	pidSetEnable = enableFlag;
}

uint8_t PID_GetEnable(void){
	return pidSetEnable;
}
