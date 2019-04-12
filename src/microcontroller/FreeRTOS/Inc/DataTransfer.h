/*
 * DataTransfer.h
 *
 *  Created on: 21.03.2019
 *      Author: Jan
 */

#ifndef DATATRANSFER_H_
#define DATATRANSFER_H_
#include <stdint.h>


//****************Debug Defines**************************************
#define FunkFernsteuer_BoardcomputerBetrieb 0		//0 --> Boardcomputer / 1 --> Funkfernsteuerung
#define SensorTaskEnable 1							//0 --> Disabled / 1 --> Enabled
#define UARTSendRaspyData 0							//0 --> Disabled / 1 --> Enabled

//Defines for FSM Debugging
#define FSMTaskEnable 1								//0 --> Disabled / 1 --> Enabled (Currently not used)
#define WuerfelerkenneUndLaden_TEST 1				//0 --> Disabled / 1 --> Enabled
//*******************************************************************



//****************UART*****************************************

typedef struct{
unsigned int startSignal 	: 1;
unsigned int roundCounter 	: 4;
unsigned int finalHSerkannt : 1;
unsigned int signalCounter	: 1;
unsigned int spareFlag		: 1;
} flags_UartData_t;


void setFlagStructure(uint8_t value);
flags_UartData_t getFlagStructure(void);
//************************************************************


//****************VL6180X**************************************
uint8_t getDistanceValue(void);
void setDistanceValue(uint8_t);
void resetDistanceValue(void);
//************************************************************



//****************MMA8451**************************************
void setXValue(uint16_t);
void setYValue(uint16_t);
void setZValue(uint16_t);

int16_t getXValue(void);
int16_t getYValue(void);
int16_t getZValue(void);

//************************************************************


//****************Radio Module**************************************
uint8_t getSpeedGroupValue(void);
uint8_t getDrehrichtung(void);
uint16_t getfinalVelocity(void);

void setDrehrichtung(uint8_t);
void setfinalVelocity(uint16_t);
void setSpeedGroupValue(uint8_t);
//************************************************************


//****************Timer extension**************************************
void incrementTimeMeasurmentValue(void);
void startTimeMeasurment(void);
uint16_t getTimeMeasurement(void);
//************************************************************

#endif /* DATATRANSFER_H_ */
