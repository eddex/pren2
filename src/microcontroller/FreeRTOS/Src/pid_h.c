/*
 * pid.c
 *
 *  Created on: 14.12.2018
 *      Author: ANDREAS
 */

#include "stm32f3xx_hal.h"
#include <motor_h.h>
#include <pid_h.h>
#include <quad_h.h>
#include <velocity_h.h>

uint8_t pidSetEnable_h = 0;

int32_t set_velo_ticks_h = 0;
int32_t meas_velo_ticks_h = 0;
int32_t integral_v_h = 0;
int32_t set_pos_ticks_h = 0;
int32_t meas_pos_ticks_h = 0;
int32_t integral_p_h = 0;
int32_t error_h = 0;
int32_t pVal_h = 0;
int32_t iVal_h = 0;
int32_t pidVal_h = 0;

// Init Routine
void PID_H_Init(){
	pidSetEnable_h = 0;
	set_velo_ticks_h = 0;
	meas_velo_ticks_h = 0;
	integral_v_h = 0;
	set_pos_ticks_h = 0;
	meas_pos_ticks_h = 0;
	integral_p_h = 0;
	error_h = 0;
	pVal_h = 0;
	iVal_h = 0;
	pidVal_h = 0;
}

// Geschwindigkeitsregler mm/s
void PID_H_Velo(int32_t set_velo){
	set_velo_ticks_h =  (set_velo * iGetriebe * TicksPerRev) / (Wirkumfang);

	meas_velo_ticks_h = Velo_H_GetVelo();
	error_h = set_velo_ticks_h - meas_velo_ticks_h;
	pVal_h = (Kp_v * error_h) / 1000;

	// Falls Geschwindigkeit 0 sein soll und 0 ist
	if(set_velo_ticks_h == 0 && meas_velo_ticks_h == 0){
		integral_v_h = 0;
	}

	integral_v_h += error_h;
	/*
	// Anti Reset Windup
	if(integral_v >= Aw_v){
		integral_v = Aw_v;
	} else if(integral_v <= -Aw_v){
		integral_v = -Aw_v;
	}
	*/
	iVal_h = (Ki_v * integral_v_h) / 1000;
	pidVal_h = pVal_h + iVal_h;
	// Begrenzung falls PWM-Wert grösser 100% wird
	if (pidVal_h > 100) {
		integral_v_h -= error_h; // Anti Reset Windup
		Motor_H_SetVelo(100);
	} else if (pidVal_h < -100) {
		integral_v_h -= error_h; // Anti Reset Windup
		Motor_H_SetVelo(-100);
	} else {
		Motor_H_SetVelo(pidVal_h);
	}
}

// Positionsregler mm
void PID_H_Pos(int32_t set_pos){
	set_pos_ticks_h = (set_pos * iGetriebe * TicksPerRev) / (Wirkumfang);

	meas_pos_ticks_h = Quad_H_GetPos();
	error_h = set_pos_ticks_h - meas_pos_ticks_h;
	pVal_h = (Kp_p * error_h) / 1000;
	integral_p_h += error_h;
	// Anti Reset Windup
	if(integral_p_h >= Aw_p){
		integral_p_h = Aw_p;
	} else if(integral_p_h <= -Aw_p){
		integral_p_h = -Aw_p;
	}
	// Falls Position erreicht Integral zurücksetzen
	if (error_h == 0) {
		integral_p_h = 0;
	}
	iVal_h = (Ki_p * integral_p_h) / 1000;
	pidVal_h = pVal_h + iVal_h;
	if (pidVal_h > MAX_VELO) {
		Motor_H_SetVelo(MAX_VELO);
	} else if (pidVal_h < -MAX_VELO) {
		Motor_H_SetVelo(-MAX_VELO);
	} else {
		Motor_H_SetVelo(pidVal_h);
	}
}

// Position erreicht
// 0 = Position nicht erreicht
// 1 = Position erreicht
uint8_t PID_H_InPos(){
	int32_t diff_h = set_pos_ticks_h-meas_pos_ticks_h;
	if (diff_h < 0){
		diff_h=diff_h*(-1);
	}
	return (diff_h < IN_POS_RANGE);
}

void PID_H_ClearError(){
	integral_v_h = 0;
	integral_p_h = 0;
}

void PID_H_SetEnable(uint8_t enableFlag){
	pidSetEnable_h = enableFlag;
}

uint8_t PID_H_GetEnable(void){
	return pidSetEnable_h;
}
