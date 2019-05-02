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
#include "DataTransfer.h"


uint8_t storeDistanceValue = 0;
uint16_t storeTimeMeasurement = 0;

taskState_t tof_erkennen(uint8_t distance){

	//Vorw�rts fahren und auf W�rfelekrennung warten, Abbruch nach 15s nichts erkennen
	storeDistanceValue = getDistanceValue();
	storeTimeMeasurement = getTimeMeasurement();


	//If time measurement exeeded 15s
	if(storeTimeMeasurement>=1499){
		Motor_Break();
		return TASK_TIME_OVERFLOW;
	}
	//W�rfel wurde erkannt
	else if((storeDistanceValue<=distance) && (storeDistanceValue>5)){
		Motor_Break();//Motoren stoppen wenn Distanz zum W�rfel im Bereich von x (mm) - y (mm) ist ODER Time overflow
		return TASK_OK;
	}
	else{
		return TASK_RUNNING;
	}
}
