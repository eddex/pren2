/*
 * RadioModule.c
 *
 *  Created on: 20.01.2019
 *      Author: Jan
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "gpio.h"
#include "usart.h"
#include "AccelSens_MMA8451.h"

uint8_t sendFlag = 0;				//Sendflag decide which AccelSens Axis value will be sent (Range from 1-5)
uint8_t speedGroupValue = 0;

//Send Data via Radio Module to PIC18
char c;								//Wird für UART Funkprotokoll verwendet. Kann Char "x", "y" oder "z" sein
uint8_t tx_dataUART1[] = {0,0};		//Send Data Buffer for Data Transfer to PIC


uint8_t finalVelocity = 0;
uint8_t drehrichtung = 0;			//Value is sent by radioControl Modul --> Reverse / Forward



uint8_t getDrehrichtung(void){
	return drehrichtung;
}

void setDrehrichtung(uint8_t uart_drehrichtung){
	drehrichtung = uart_drehrichtung;
}

uint8_t getfinalVelocity(void){
	return finalVelocity;
}

void setfinalVelocity(uint8_t velocity){
	finalVelocity = velocity;
}

uint8_t getSpeedGroupValue(void){
	return speedGroupValue;
}

void setSpeedGroupValue(uint8_t groupValue){
	speedGroupValue = groupValue;
}

void sendSensorDatatoRadioModule(void){

	  //sendFlag decides which Axis value will be transmitted to PIC
	  if(sendFlag ==0){
		  sendFlag+=1;
		  c='z';
		  //HAL_UART_Transmit(&huart1,(uint8_t*)&c, 1,1000);
		  //Measure all 3 Axis values
		  measureAccel3AxisValues();

		  //Distance is disabled because of Debugging problems!!!
		  //measureDistanceValue();
	  }


	  else if(sendFlag == 1){
		  sendFlag+=1;

		  //Send HighByte of AccelValue
		  tx_dataUART1[0] = (uint8_t) (getZValue() >> 8);

		  //Send LowByte of AccelValue
		  tx_dataUART1[1] = (uint8_t) (getZValue() & 0xff);
		  if(tx_dataUART1[1] == 122||tx_dataUART1[1] == 121||tx_dataUART1[1] == 120||tx_dataUART1[1] == 100){
			  tx_dataUART1[1]=123;
		  }
		  //HAL_UART_Transmit(&huart1, tx_dataUART1, 2, 1000);

		  //UART2 Debug
		  //**************************************
		  //sendFlag=0;
		  //HAL_UART_Transmit(&huart2,(uint8_t*)&c, 1,1000);
		  //HAL_UART_Transmit(&huart2, tx_dataUART1, 2, 1000);
		  //***************************************
	  }



	  else if(sendFlag ==2){
		  sendFlag+=1;
		  c='y';
		  //HAL_UART_Transmit(&huart1,(uint8_t*)&c, 1,1000);
	  }

	  else if(sendFlag == 3){
		  sendFlag+=1;

		  //Send HighByte of AccelValue
		  tx_dataUART1[0] = (uint8_t) (getYValue() >> 8);

		  //Send LowByte of AccelValue
		  tx_dataUART1[1] = (uint8_t) (getYValue() & 0xff);
		  if(tx_dataUART1[1] == 122||tx_dataUART1[1] == 121||tx_dataUART1[1] == 120||tx_dataUART1[1] == 100){
			  tx_dataUART1[1]=123;
		  }
		  //HAL_UART_Transmit(&huart1, tx_dataUART1, 2, 1000);

		  //UART2 Debug
		  //**************************************
		  //sendFlag=0;
		  //HAL_UART_Transmit(&huart2,(uint8_t*)&c, 1,1000);
		  //HAL_UART_Transmit(&huart2, tx_dataUART1, 2, 1000);
		  //***************************************
	  }

	  else if(sendFlag ==4){
		  sendFlag+=1;
		  c='x';
		  //HAL_UART_Transmit(&huart1,(uint8_t*)&c, 1,1000);
	  }


	  else if(sendFlag == 5){
		  sendFlag+=1;

		  //Send HighByte of AccelValue
		  tx_dataUART1[0] = (uint8_t) (getXValue() >> 8);

		  //Send LowByte of AccelValue
		  tx_dataUART1[1] = (uint8_t) (getXValue() & 0xff);
		  if(tx_dataUART1[1] == 122||tx_dataUART1[1] == 121||tx_dataUART1[1] == 120||tx_dataUART1[1] == 100){
			  tx_dataUART1[1]=119;
		  }
		  //HAL_UART_Transmit(&huart1, tx_dataUART1, 2, 1000);

		  sendFlag=0;

		  //UART2 Debug
		  //**************************************
		  //sendFlag=0;
		  //HAL_UART_Transmit(&huart2,(uint8_t*)&c, 1,1000);
		  //HAL_UART_Transmit(&huart2, tx_dataUART1, 2, 1000);
		  //***************************************
	  }

	  /*
	  //Send Distance Value
	  else if(sendFlag==6){
		  sendFlag+=1;
		  c='d';
		  HAL_UART_Transmit(&huart1,(uint8_t*)&c, 1,1000);
	  }

	  else if(sendFlag==7){
		  sendFlag=0;
		  tx_dataUART1[1]=0;
		  tx_dataUART1[0] = getDistanceValue();
		  if(tx_dataUART1[0] == 122||tx_dataUART1[0] == 121||tx_dataUART1[0] == 120||tx_dataUART1[0] == 100){
			 tx_dataUART1[0]=119;
		  }
		  HAL_UART_Transmit(&huart1, tx_dataUART1, 1, 1000);

		  //UART2 Debug
		  //sendFlag=0;
		  HAL_UART_Transmit(&huart2,(uint8_t*)&c, 1,1000);
		  HAL_UART_Transmit(&huart2, tx_dataUART1, 1, 1000);
	  }*/


}
