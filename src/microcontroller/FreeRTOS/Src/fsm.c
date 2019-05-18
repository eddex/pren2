/*
 * fsm.c
 *
 *  Created on: 02.03.2019
 *      Author: Jan
 */

#include <motor_v.h>
#include <motor_h.h>
#include <pid_v.h>
#include <pid_h.h>
#include <quad_v.h>
#include <quad_h.h>
#include <velocity_v.h>
#include <velocity_h.h>
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
#include "fsm.h"
#include "DataTransfer.h"


uint8_t storeDistanceValue = 0;
uint16_t storeTimeMeasurement = 0;

taskState_t tof_erkennen(uint8_t distance){

	//Vorwärts fahren und auf Würfelekrennung warten, Abbruch nach 15s nichts erkennen
	storeDistanceValue = getDistanceValue();
	storeTimeMeasurement = getTimeMeasurement();


	//If time measurement exeeded 15s
	if(storeTimeMeasurement>=1499){
		Motor_V_Break();
		Motor_H_Break();
		return TASK_TIME_OVERFLOW;
	}
	//Würfel wurde erkannt
	else if((storeDistanceValue<=distance) && (storeDistanceValue>5)){
		Motor_V_Break();//Motoren stoppen wenn Distanz zum Würfel im Bereich von x (mm) - y (mm) ist ODER Time overflow
		Motor_H_Break();
		return TASK_OK;
	}
	else{
		return TASK_RUNNING;
	}
}
