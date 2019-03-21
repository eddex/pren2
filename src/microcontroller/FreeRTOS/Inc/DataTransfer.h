/*
 * DataTransfer.h
 *
 *  Created on: 21.03.2019
 *      Author: Jan
 */

#ifndef DATATRANSFER_H_
#define DATATRANSFER_H_

#include <stdint.h>


//****************UART**************************************

typedef struct{
unsigned int startSignal 	: 1;
unsigned int RundenCounter 	: 4;
unsigned int FinalHSerkannt : 1;
unsigned int singalCounter	: 1;
unsigned int spareFlag		: 1;
} flags_UartData_t;


void setFlagStructure(uint8_t value);
flags_UartData_t getFlagStructure(void);
//************************************************************


//****************VL6180X**************************************
uint8_t getDistanceValue(void);
void setDistanceValue(uint8_t);
//************************************************************



//****************MMA8451**************************************
void setXValue(uint16_t);
void setYValue(uint16_t);
void setZValue(uint16_t);

int16_t getXValue(void);
int16_t getYValue(void);
int16_t getZValue(void);

void setEnableSensorTask(uint8_t);
uint8_t getEnableSensorTask(void);
//************************************************************


//****************Radio Module**************************************
uint8_t getSpeedGroupValue(void);
uint8_t getDrehrichtung(void);
uint16_t getfinalVelocity(void);

void setDrehrichtung(uint8_t);
void setfinalVelocity(uint16_t);
void setSpeedGroupValue(uint8_t);
//************************************************************


#endif /* DATATRANSFER_H_ */
