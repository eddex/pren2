/*
 * fsm.c
 *
 *  Created on: 02.03.2019
 *      Author: Jan
 */

#include "main.h"
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "RadioModule.h"
#include "AccelSens_MMA8451.h"
#include "DistSens_VL6180X.h"
#include "velocity.h"
#include "quad.h"
#include "motor.h"
#include "pid.h"
#include "fsm.h"


//enum stateOfTask taskState;
uint16_t storeTimeValue= 0;

taskState wurfel_erkennen(void){

	startTimeMeasurment();											//Zeitmessung beginnen f�r Abbruchkriterium des Tasks

	PID_Velo(12);													//Motoren starten auf tiefster Geschwindigkeitsstufe


	//Vorw�rts fahren und auf W�rfelekrennung warten, Abbruch nach 15s nichts erkennen
	while(getDistanceValue() > 60 || getTimeMeasurement()<1500){
		storeTimeValue=getTimeMeasurement();
	}
	Motor_Break();													//Motoren stoppen wenn Distanz zum W�rfel im Bereich von x (mm) - y (mm) ist ODER Time overflow

	if(storeTimeValue>1500){return TASK_TIME_OVERFLOW;}
	else{return TASK_OK;}
}

taskState haltesignal_erkennen(void){
	startTimeMeasurment();											//Zeitmessung beginnen f�r Abbruchkriterium des Tasks

	PID_Velo(12);													//Motoren starten auf tiefster Geschwindigkeitsstufe

	//Vorw�rts fahren und auf W�rfelekrennung warten, Abbruch nach 15s nichts erkennen
	while(getDistanceValue() > 60 || getTimeMeasurement()<1500){
		storeTimeValue=getTimeMeasurement();
	}
	Motor_Break();													//Motoren stoppen wenn Distanz zum W�rfel im Bereich von x (mm) - y (mm) ist ODER Time overflow

	if(storeTimeValue>1500){return TASK_TIME_OVERFLOW;}
	else{return TASK_OK;}
}
