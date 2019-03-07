/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "AccelSens_MMA8451.h"
#include "DistSens_VL6180X.h"
#include "usart.h"
#include "RadioModule.h"
#include "tim.h"
#include "motor.h"
#include "pid.h"
#include "velocity.h"
#include "quad.h"
#include "fsm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId SensorTaskHandle;
osThreadId UartRadioTaskHandle;
osThreadId ServoTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void StartTask04(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of SensorTask */
  osThreadDef(SensorTask, StartTask02, osPriorityNormal, 0, 128);
  SensorTaskHandle = osThreadCreate(osThread(SensorTask), NULL);

  /* definition and creation of UartRadioTask */
  osThreadDef(UartRadioTask, StartTask03, osPriorityNormal, 0, 128);
  UartRadioTaskHandle = osThreadCreate(osThread(UartRadioTask), NULL);

  /* definition and creation of ServoTask */
  osThreadDef(ServoTask, StartTask04, osPriorityNormal, 0, 128);
  ServoTaskHandle = osThreadCreate(osThread(ServoTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  enum fsm fsm_state; // create enum for statemachine task
  fsm_state = STARTUP; // Default Task -> Startup

  /* Infinite loop */
  for(;;)
  {
	  //Statemachine acc. to PREN1 Documentation p.24
	  switch(fsm_state){
	  case STARTUP:
		  	  // hallo Jan
		  	  break;
	  case WURFEL_ERKENNEN:
		  	  	  	  	  	  	  if(wurfel_erkennen()==TASK_OK){
		  	  	  	  	  	  		  fsm_state = WURFEL_LADEN;
		  	  	  	  	  	  	  }
		  	  	  	  	  	  	  else{
		  	  	  	  	  	  		  //Was machen wir wenn der Würfel nicht erkannt wird???
		  	  	  	  	  	  	  }
	  		  break;
	  case WURFEL_LADEN:
	  		  break;
	  case WURFEL_GELADEN:
	  		  break;
	  case START_ERKENNEN:
	  	  		  break;
	  case SCHNELLFAHRT:
	  		  break;
	  }
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  uint8_t txData[3];
  //char x = 'x';
  //char y = 'y';
  char z = 'z';
  /* Infinite loop */
  for(;;)
  {
	  //Im Main.c gibt es bei der Initialiserung eine Funktion, die Entscheidet, ob dieser Task ausgeführt wird oder nicht
	  if(getEnableSensorTask() == 1){
		  measureAccel3AxisValues();
		  //measureDistanceValue();
		  txData[0] = (uint8_t) z;
		  txData[1] = (uint8_t) (getZValue() >> 8);
		  txData[2] = (uint8_t) (getZValue() & 0xff);

		  //txData[0] = getDistanceValue();
		  HAL_UART_Transmit(&huart2, txData, 3, 100);
		  osDelay(2000);
	  }

	  else{
		  osDelay(1000);
	  }

  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
	uint8_t firstForwardCount =0;		//Bremst motor vor Seitenwechsel
	uint8_t firstReverseCount = 0;		//dito
  /* Infinite loop */
  for(;;)
  {

	  	  //Drehrichtung FORWARD
	  	  if(getDrehrichtung() == 108){

	  		  firstReverseCount = 0;

	  		  //Break function
	  		  if(firstForwardCount==0){
	  			  firstForwardCount++;
	  			  Motor_Break();
	  			  osDelay(1000);

	  		  }

	  		  else{													//Drive with UART1 received pwmValue
	  			  if(getPidEnable()==1){
	  				  PID_Velo((-1)*getfinalVelocity());
	  				  setPidEnable(0);
	  			  }
	  		  }
	  	  }

	  	  	  //Drehrichtung REVERSE
	  	  else if(getDrehrichtung() == 114){
	  		  firstForwardCount = 0;

	  		  //Break function
	  		 if(firstReverseCount==0){
	  			  firstReverseCount++;
	  			  Motor_Break();
	  			  osDelay(1000);
	  		 }


	  		 else{												//Drive with UART1 received pwmVlue

	  				  //Nur alle 10ms!
	  			if(getPidEnable()==1){
	  				  PID_Velo(getfinalVelocity());
	  				  setPidEnable(0);
	  			  }
	  		  }
	  		}


	  	//Imlement a Error Handler witch sets de "drehrichtung" value to 0, if we dont receive Interrupts from UART1
	  	//--> Not implemented yet
	  	else{
	  		 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
	  		 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);

	  	 }



  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the ServoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void const * argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
	/*
	   * Servo Angle-Control:
	   * -90°	=> 1.1ms (PWM_Value = 11 * (480'000Counts / 200) = 26'400
	   * 0°		=> 1.5ms (PWM_Value = 15 * (480'000Counts / 200) = 36'000
	   * 90°	=> 1.9ms (PWM_Value = 19 * (480'000Counts / 200) = 45'600
	 */

  uint16_t servoPWM = 0;

  for(;;)
  {
	  servoPWM = 26400 + getSpeedGroupValue()*2740;
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, servoPWM);
	  osDelay(10);
  }
  /* USER CODE END StartTask04 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
