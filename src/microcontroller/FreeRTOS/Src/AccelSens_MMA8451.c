/*
 * AccelSens_MMA8451.c
 *
 *  Created on: 22.12.2018
 *      Author: Jan
 */



/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "i2c.h"
#include "gpio.h"

#include "AccelSens_MMA8451.h"


//Global variables
uint16_t MMA8451_DevAddress = finalDevAddress_MMA8451<<1;			//MMA8451 Address shiftet for I2C use
uint8_t dataBuffer[2];												//Databuffer for Init of Device (I2C)
uint8_t axisXYZ_dataBuffer[] = {0,0,0,0,0,0};					    //Databuffer for Accel Values of Devide (I2C)

uint8_t enableSensorTask = 0;

//Values of the X_Y_Z_Register of the accel Sensor in 2er Complement
volatile int16_t Xout_14_bit = 0;
volatile int16_t Yout_14_bit = 0;
volatile int16_t Zout_14_bit = 0;

//Values with the corresponding value in mg
volatile int16_t Xout_g = 0;
volatile int16_t Yout_g = 0;
volatile int16_t Zout_g = 0;


void setEnableSensorTask(uint8_t enable){
	enableSensorTask = enable;
}

uint8_t getEnableSensorTask(void){
	return enableSensorTask;
}

void MMA8451_Init(void){

	//Reset databuffer for Init routine
	dataBuffer[0] = 0;
	dataBuffer[1] = 0;

	//Reset MMA8451
	dataBuffer[0] = 0x40; //Set the Reset Bit for Device
	if(HAL_I2C_Mem_Write(&hi2c1, MMA8451_DevAddress, MMA8451_REG_CTRL_REG2,1, dataBuffer, 1, 100)==HAL_OK);


	//Wait while reset bit is set(automatically cleard after reset by MC)
	dataBuffer[0] = 0;
	do{
		if(HAL_I2C_Mem_Read(&hi2c1, MMA8451_DevAddress,MMA8451_REG_CTRL_REG2,1, dataBuffer, 1, 100)==HAL_OK);
	}while(dataBuffer[0] != 0);


	//Check Data in MMA8451_REG_XYZ_DATA_CFG
	dataBuffer[0] = 0;
	dataBuffer[1] = 0;
	if(HAL_I2C_Mem_Read(&hi2c1, MMA8451_DevAddress,MMA8451_REG_XYZ_DATA_CFG,1, dataBuffer, 1, 100)==HAL_OK);

	//Output buffer data format full scale. Default value: 00 (2g) --> TestLed for Debug
	uint8_t test = dataBuffer[0] && 0x03;
	if(test == 0x00){
		//HAL_GPIO_WritePin(HB_Sleep_GPIO_Port, HB_Sleep_Pin, GPIO_PIN_RESET);
	}

	//Configuration of System Control Register 2
	dataBuffer[0] = 0x02;					//Choose active MODE
	dataBuffer[1] = 0;
	if(HAL_I2C_Mem_Write(&hi2c1, MMA8451_DevAddress,MMA8451_REG_CTRL_REG2,1, dataBuffer, 1, 100)==HAL_OK);

	//Configuration of System Control Register 1
	dataBuffer[0] = 0x3D;
	dataBuffer[1] = 0;
	if(HAL_I2C_Mem_Write(&hi2c1, MMA8451_DevAddress,MMA8451_REG_CTRL_REG1,1, dataBuffer, 1, 100)==HAL_OK);

	//Check new Data in System Control Register 1
	dataBuffer[0] = 0;
	dataBuffer[1] = 0;
	if(HAL_I2C_Mem_Read(&hi2c1, MMA8451_DevAddress,MMA8451_REG_CTRL_REG1,1, dataBuffer, 1, 100)==HAL_OK);

	if(dataBuffer[0] == 0x3D){
		//HAL_GPIO_WritePin(HB_Sleep_GPIO_Port, HB_Sleep_Pin, GPIO_PIN_SET);
	 }
	//HAL_GPIO_WritePin(HB_Sleep_GPIO_Port, HB_Sleep_Pin, GPIO_PIN_RESET);
}



void measureAccel3AxisValues(void){

	//Read 6 Bytes --> 2Byte value of each axis
	if(HAL_I2C_Mem_Read(&hi2c1, MMA8451_DevAddress,MMA8451_REG_OUT_X_MSB,1, axisXYZ_dataBuffer, 6, 10)==HAL_OK);

	//Shift bits because of 14bit value --> See datasheet
	Xout_14_bit = ((axisXYZ_dataBuffer[0]<<8 | axisXYZ_dataBuffer[1])) >> 2;           // Compute 14-bit X-axis output value
	Yout_14_bit = ((axisXYZ_dataBuffer[2]<<8 | axisXYZ_dataBuffer[3])) >> 2;           // Compute 14-bit Y-axis output value
	Zout_14_bit = ((axisXYZ_dataBuffer[4]<<8 | axisXYZ_dataBuffer[5])) >> 2;           // Compute 14-bit Z-axis output value

	//If MSB is '1' --> 2er Complement: Negative value
	if((Xout_14_bit & 0x2000) == 0x2000){
		Xout_14_bit = (-1)*Xout_14_bit;
		Xout_14_bit = Xout_14_bit & 0x3FFF;
		Xout_14_bit += 1;
	}

	//If MSB is '1' --> 2er Complement: Negative value
	if((Yout_14_bit & 0x2000) == 0x2000){
		Yout_14_bit = (-1)*Yout_14_bit;
		Yout_14_bit = Yout_14_bit & 0x3FFF;
		Yout_14_bit += 1;
	}

	//If MSB is '1' --> 2er Complement: Negative value
	if((Zout_14_bit & 0x2000) == 0x2000){
		Zout_14_bit = (-1)*Zout_14_bit;
		Zout_14_bit = Zout_14_bit & 0x3FFF;
		Zout_14_bit += 1;
	}

	//Calculate the final Value in [mg] in depending on the sensitivity
	Xout_g = (1000*Xout_14_bit) / SENSITIVITY_2G;              // Compute X-axis output value in mg's
	Yout_g = (1000*Yout_14_bit) / SENSITIVITY_2G;              // Compute Y-axis output value in mg's
	Zout_g = (1000*Zout_14_bit) / SENSITIVITY_2G;              // Compute Z-axis output value in mg's
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
