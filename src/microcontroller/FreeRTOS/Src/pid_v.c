/*
 * pid.c
 *
 *  Created on: 14.12.2018
 *      Author: ANDREAS
 */
#include "stm32f3xx_hal.h"
#include <motor_v.h>
#include <pid_v.h>
#include <quad_v.h>
#include <velocity_v.h>

uint8_t pidSetEnable_v = 0;

int32_t set_velo_ticks_v = 0;
int32_t meas_velo_ticks_v = 0;
int32_t integral_v_v = 0;
int32_t set_pos_ticks_v = 0;
int32_t meas_pos_ticks_v = 0;
int32_t integral_p_v = 0;
int32_t error_v = 0;
int32_t pVal_v = 0;
int32_t iVal_v = 0;
int32_t pidVal_v = 0;

// Init Routine
void PID_V_Init(){
	pidSetEnable_v = 0;
	set_velo_ticks_v = 0;
	meas_velo_ticks_v = 0;
	integral_v_v = 0;
	set_pos_ticks_v = 0;
	meas_pos_ticks_v = 0;
	integral_p_v = 0;
	error_v = 0;
	pVal_v = 0;
	iVal_v = 0;
	pidVal_v = 0;
}

// Geschwindigkeitsregler mm/s
void PID_V_Velo(int32_t set_velo){
	set_velo_ticks_v =  (set_velo * iGetriebe * TicksPerRev) / (Wirkumfang);

	meas_velo_ticks_v = Velo_V_GetVelo();
	error_v = set_velo_ticks_v - meas_velo_ticks_v;
	pVal_v = (Kp_v * error_v) / 1000;

	// Falls Geschwindigkeit 0 sein soll und 0 ist
	if(set_velo_ticks_v == 0 && meas_velo_ticks_v == 0){
		integral_v_v = 0;
	}

	integral_v_v += error_v;
	/*
	// Anti Reset Windup
	if(integral_v >= Aw_v){
		integral_v = Aw_v;
	} else if(integral_v <= -Aw_v){
		integral_v = -Aw_v;
	}
	*/
	iVal_v = (Ki_v * integral_v_v) / 1000;
	pidVal_v = pVal_v + iVal_v;
	// Begrenzung falls PWM-Wert grösser 100% wird
	if (pidVal_v > 100) {
		integral_v_v -= error_v; // Anti Reset Windup
		Motor_V_SetVelo(100);
	} else if (pidVal_v < -100) {
		integral_v_v -= error_v; // Anti Reset Windup
		Motor_V_SetVelo(-100);
	} else {
		Motor_V_SetVelo(pidVal_v);
	}
}

// Positionsregler mm
void PID_V_Pos(int32_t set_pos){
	set_pos_ticks_v = (set_pos * iGetriebe * TicksPerRev) / (Wirkumfang);

	meas_pos_ticks_v = Quad_V_GetPos();
	error_v = set_pos_ticks_v - meas_pos_ticks_v;
	pVal_v = (Kp_p * error_v) / 1000;
	integral_p_v += error_v;
	// Anti Reset Windup
	if(integral_p_v >= Aw_p){
		integral_p_v = Aw_p;
	} else if(integral_p_v <= -Aw_p){
		integral_p_v = -Aw_p;
	}
	// Falls Position erreicht Integral zurücksetzen
	if (error_v == 0) {
		integral_p_v = 0;
	}
	iVal_v = (Ki_p * integral_p_v) / 1000;
	pidVal_v = pVal_v + iVal_v;
	if (pidVal_v > MAX_VELO) {
		Motor_V_SetVelo(MAX_VELO);
	} else if (pidVal_v < -MAX_VELO) {
		Motor_V_SetVelo(-MAX_VELO);
	} else {
		Motor_V_SetVelo(pidVal_v);
	}
}

// Position erreicht
// 0 = Position nicht erreicht
// 1 = Position erreicht
uint8_t PID_V_InPos(){
	int32_t diff_v = set_pos_ticks_v-meas_pos_ticks_v;
	if (diff_v < 0){
		diff_v=diff_v*(-1);
	}
	return (diff_v < IN_POS_RANGE);
}

void PID_V_ClearError(){
	integral_v_v = 0;
	integral_p_v = 0;
}

void PID_V_SetEnable(uint8_t enableFlag){
	pidSetEnable_v = enableFlag;
}

uint8_t PID_V_GetEnable(void){
	return pidSetEnable_v;
}
