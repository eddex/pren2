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
volatile uint8_t debugDistanceValue = 0;

taskState_t wurfel_erkennen(void){

	startTimeMeasurment();											//Zeitmessung beginnen für Abbruchkriterium des Tasks

	PID_Velo(12);													//Motoren starten auf tiefster Geschwindigkeitsstufe


	//Vorwärts fahren und auf Würfelekrennung warten, Abbruch nach 15s nichts erkennen
	while((getDistanceValue() > 60 && getTimeMeasurement()<1500) || (getDistanceValue()<5)){		//OR Abfrage, da Sensor zu beginn manchmal 0 zurückliefert
		storeTimeValue=getTimeMeasurement();
		debugDistanceValue = getDistanceValue();
	}
	Motor_Break();													//Motoren stoppen wenn Distanz zum Würfel im Bereich von x (mm) - y (mm) ist ODER Time overflow

	if(storeTimeValue>=1499){return TASK_TIME_OVERFLOW;}
	else{return TASK_OK;}
}

taskState_t haltesignal_erkennen(void){
	startTimeMeasurment();											//Zeitmessung beginnen für Abbruchkriterium des Tasks

	PID_Velo(12);													//Motoren starten auf tiefster Geschwindigkeitsstufe

	//Vorwärts fahren und auf Würfelekrennung warten, Abbruch nach 15s nichts erkennen
	while(getDistanceValue() > 60 || getTimeMeasurement()<1500){
		storeTimeValue=getTimeMeasurement();
	}
	Motor_Break();													//Motoren stoppen wenn Distanz zum Würfel im Bereich von x (mm) - y (mm) ist ODER Time overflow

	if(storeTimeValue>1500){return TASK_TIME_OVERFLOW;}
	else{return TASK_OK;}
}
