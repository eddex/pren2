/*
 * DistSens_VL6180X.c
 *
 *  Created on: 22.12.2018
 *      Author: Jan
 */


/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "i2c.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"

#include "DistSens_VL6180X.h"
#include "DataTransfer.h"

taskState_t distTaskState;
uint8_t dataBuffer[] ={0,0};
uint16_t VL6180X_DevAddress = finalDevAddress_VL6180X<<1;			//MMA8451 Address shiftet for I2C use



taskState_t VL6180X_Init(void){

	distTaskState = TASK_OK;

	//Wait while reset bit is set(automatically cleard after reset by MC)
	//Register System_fresh_out_of_Reset (bit0: default of 1, user can set this to 0 after initial boot and can therefore use this to chef for a reset condition)

	dataBuffer[0] = 0;
	do{
		if(HAL_I2C_Mem_Read(&hi2c1, VL6180X_DevAddress,0x0016,2, dataBuffer, 1, 100)== HAL_OK){
			//HAL_GPIO_WritePin(HBridgeEnable_GPIO_Port, HBridgeEnable_Pin, GPIO_PIN_SET);
		}
		else{distTaskState = TASK_ERROR;};
	}while((dataBuffer[0] & 0x01) == 0);

	//Tuning settings refer to appendix 1 of Application Note
	dataBuffer[0] = 0x01;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x0207,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x01;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x0208,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x00;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x0096,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0xFD;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x0097,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x00;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x00e3,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x04;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x00e4,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x02;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x00e5,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x01;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x00e6,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x03;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x00e7,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x02;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x00f5,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x05;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x00D9,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0xce;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x00DB,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x03;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x00DC,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0xF8;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x00DD,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x00;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x009f,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x3c;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x00a3,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x00;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x00b7,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x3c;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x00bb,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x09;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x00b2,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x09;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x00ca,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x01;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x0198,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};

	dataBuffer[0] = 0x17;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x01b0,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x00;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x01ad,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x05;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x00FF,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x05;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x0100,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x05;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x0199,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x1b;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x01a6,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x3e;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x01ac,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x1f;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x01a7,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x00;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x0030,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};

	dataBuffer[0] = 0x10;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x0011,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x30;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x010a,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x46;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x003f,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0xFF;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x0031,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x63;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x0040,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x01;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x002e,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x09;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x001b,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x31;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x003e,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
	dataBuffer[0] = 0x24;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x0014,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};


	/*
	//Read Reg 0x014 and after write 0b100 to bits [2:0]
	dataBuffer[0] = 0;
	HAL_I2C_Mem_Read(&hi2c1, VL6180X_DevAddress,0x14,1, dataBuffer, 1, 100);
	dataBuffer[0] = dataBuffer[0] & 0xFC; //Clear Bit 0 and 1
	dataBuffer[0] = dataBuffer[0] | 0x04; //Set Bit 2 to 1
	HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress,0x14,1, dataBuffer, 1, 100);
	*/

	//Sysrange_start --> Ranging Mode (single-shot)
	//dataBuffer[0] = 0x01;

	//Sysrange_start --> Ranging Mode (continous)
	dataBuffer[0] = 0x03;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x0018,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};


	return distTaskState;
}





taskState_t measureDistanceValue(void){
	distTaskState = TASK_OK;
	do{
		dataBuffer[0] = 0;
		if(HAL_I2C_Mem_Read(&hi2c1, VL6180X_DevAddress,0x004F,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};
		//dataBuffer[0] = dataBuffer[0] & 0x07; //The 3 lowests bits are relevant
	}while((dataBuffer[0] & 0x04) != 0x04);


	//Finally read range value
	dataBuffer[0] = 0;
	if(HAL_I2C_Mem_Read(&hi2c1, VL6180X_DevAddress,0x0062,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};

	//Store the measured Distance Value
	setDistanceValue(dataBuffer[0]);

	//Clear interrupt status flag
	dataBuffer[0] = 0x07;
	if(HAL_I2C_Mem_Write(&hi2c1, VL6180X_DevAddress, 0x0015,2, dataBuffer, 1, 100)==HAL_OK);else{distTaskState = TASK_ERROR;};

	/*if(distanceValue < 100 && distanceValue >80){
	   HAL_GPIO_WritePin(BlueTestLed_GPIO_Port, BlueTestLed_Pin, GPIO_PIN_SET);
	}
	else{
	   HAL_GPIO_WritePin(BlueTestLed_GPIO_Port, BlueTestLed_Pin, GPIO_PIN_RESET);
	}*/

	return distTaskState;
}


