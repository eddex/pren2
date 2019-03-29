/*
 * DataTransfer.c
 *
 *  Created on: 21.03.2019
 *      Author: Jan
 */

#include "DataTransfer.h"
#include <stdint.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"


//****************UART**************************************
flags_UartData_t flags_UartData;

void setFlagStructure(uint8_t value){

	flags_UartData.startSignal = (value & 0b10000000)>>7;
	flags_UartData.finalHSerkannt = (value & 0b00000100)>>2;
	flags_UartData.signalCounter = (value & 0b00000010)>>1;
	flags_UartData.spareFlag = (value & 0b00000001);
	flags_UartData.roundCounter = (value & 0b01111000)>>3;
}

flags_UartData_t getFlagStructure(void){
	return flags_UartData;
}
//************************************************************




//****************VL6180X**************************************
uint8_t distanceValue = 0; //Default out of range

uint8_t getDistanceValue(void){
	uint8_t distanceCopy;
	taskENTER_CRITICAL();
	distanceCopy = distanceValue;
	taskEXIT_CRITICAL();
	return distanceCopy;
}

void setDistanceValue(uint8_t value){
	taskENTER_CRITICAL();
	distanceValue = value;
	taskEXIT_CRITICAL();
}
//************************************************************



//****************MMA8451**************************************
//Values with the corresponding value in mg
int16_t Xout_g = 0;
int16_t Yout_g = 0;
int16_t Zout_g = 0;

void setXValue(uint16_t value){
	taskENTER_CRITICAL();
	Xout_g = value;
	taskEXIT_CRITICAL();
}
void setYValue(uint16_t value){
	taskENTER_CRITICAL();
	Yout_g = value;
	taskEXIT_CRITICAL();
}
void setZValue(uint16_t value){
	taskENTER_CRITICAL();
	Zout_g = value;
	taskEXIT_CRITICAL();
}

int16_t getXValue(void){
	return Xout_g;
}

int16_t getYValue(void){
	return Yout_g;
}

int16_t getZValue(void){
	return Zout_g;
}
//************************************************************




//****************Radio Module**************************************
uint16_t finalVelocity = 0;
uint8_t drehrichtung = 0;			//Value is sent by radioControl Modul --> Reverse / Forward
uint8_t speedGroupValue = 0;


uint8_t getDrehrichtung(void){
	return drehrichtung;
}

void setDrehrichtung(uint8_t uart_drehrichtung){
	drehrichtung = uart_drehrichtung;
}

uint16_t getfinalVelocity(void){
	return finalVelocity;
}

void setfinalVelocity(uint16_t velocity){
	finalVelocity = velocity;
}

uint8_t getSpeedGroupValue(void){
	return speedGroupValue;
}

void setSpeedGroupValue(uint8_t groupValue){
	speedGroupValue = groupValue;
}
//************************************************************



//****************Timer extension**************************************
uint16_t timeMeasurementValue = 0;

void incrementTimeMeasurmentValue(){
	timeMeasurementValue++;
}

void startTimeMeasurment(){
	timeMeasurementValue = 0;
}

uint16_t getTimeMeasurement(){
	uint16_t copytimeMeasurementValue;
	taskENTER_CRITICAL();
	copytimeMeasurementValue = timeMeasurementValue;
	taskEXIT_CRITICAL();
	return copytimeMeasurementValue;
}
//************************************************************
