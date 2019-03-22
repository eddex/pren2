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
#include "servo.h"
#include "fsm.h"
#include "DataTransfer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SlowVelo 100 // langsame Geschwindigkeit [mm/s]
#define DistTofToWurfel 100 // Distanz zwischen Tof und Würfel
#define WurfelLength 50 // Würfellänge
#define MaxVelo 2000 // maximale Geschwindigkeit [mm/s]
#define MaxNbrSignals 10 // maximale Anzahl Signale auf der Strecke
#define MaxNbrRounds 2 // maximale Anzahl Runden
#define MaxLoadAttempts 4 // maximale Anzahl Würfelladeversuche
#define MaxTrackLength 15000 // maximale Streckenlänge [mm]


#define WuerfelerkenneUndLaden_TEST 1
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
osSemaphoreId myBinarySem01Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void StartTask04(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationIdleHook(void);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 1 */
uint32_t TickCounter = 0;
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{
	TickCounter = 0;
}

__weak unsigned long getRunTimeCounterValue(void)
{
	return TickCounter++;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 2 */
__weak void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 5 */
__weak void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

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

  /* Create the semaphores(s) */
  /* definition and creation of myBinarySem01 */
  osSemaphoreDef(myBinarySem01);
  myBinarySem01Handle = osSemaphoreCreate(osSemaphore(myBinarySem01), 1);

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
  fsm_state = STARTUP; // Default State -> Startup

  taskState_t taskState;
  taskState = TASK_RUNNING;

  uint8_t wurfelCtr = 0; // Zähler Anzahl Ladeversuche

  int32_t posStart = 0;	// Positionsmerker bei Startsignal
  int32_t posWurfel = 0; // Positionsmerker Würfel erkannt
  int32_t posHaltesignal = 0; // Positionsmerker Haltesignal erkannt
  int32_t distHaltesignal = 0; // Distanz zum Haltesignal

  uint32_t servoCtr = 0; // Zähler um Servo langsam zu bewegen
  uint32_t accCtr = 0; // Zähler um Motoren langsam zu beschleunigen

  uint8_t servoAngle = 0; // Winkelmerker Servo
  uint16_t speed = 0; // Geschwindigkeit der Motoren

  /* Infinite loop */
  for(;;)
  {


	//Statemachine acc. to PREN1 Documentation p.24
	switch(fsm_state){
	// Warten auf Startbefehl von Raspi
	case STARTUP:
		// TODO implement method getStartSignal in Raspi.c
		//getStartSignal is available with the function_call "flags_UartData_t getFlagStructure(void)" see for more info in usart.h
		if (0) /*getStartSignal()*/{
			posStart=Quad_GetPos();
			fsm_state = WURFEL_ERKENNEN;
			startTimeMeasurment();											//Zeitmessung beginnen für Abbruchkriterium des Tasks
			PID_Velo(100);													//Motoren starten auf tiefster Geschwindigkeitsstufe
		}

		#if WuerfelerkenneUndLaden_TEST
			fsm_state = WURFEL_ERKENNEN;
			startTimeMeasurment();											//Zeitmessung beginnen für Abbruchkriterium des Tasks
			PID_Velo(100);													//Motoren starten auf tiefster Geschwindigkeitsstufe
		#endif


	break;

	// Warten bis Würfel erkennt wird
	case WURFEL_ERKENNEN:
		//Nicht notwendig, da in Funktion wurfel_erkennen() bereits gemacht wird.
		//PID_Velo(SlowVelo); // mit langsamer Geschwindigkeit fahren

		taskState = wurfel_erkennen();

		if(taskState == TASK_OK){
			posWurfel = Quad_GetPos();
			fsm_state = WURFEL_VORFAHREN;
			HAL_GPIO_WritePin(LED_Heartbeat_GPIO_Port, LED_Heartbeat_Pin, GPIO_PIN_SET);
		}
		else if(taskState == TASK_TIME_OVERFLOW){ // Würfel nicht erkannt
			fsm_state = STARTPOSITION;
			HAL_GPIO_WritePin(LED_Heartbeat_GPIO_Port, LED_Heartbeat_Pin, GPIO_PIN_SET);
		}
		else{}

		break;

	// Vorfahren mit Lademechanismus zum Würfel
	case WURFEL_VORFAHREN:
		if(0){
			PID_Pos(posWurfel+DistTofToWurfel); // An Würfelposition fahren
		}


		// TODO implement method PID_InPos in pid.c
		if (0) /*PID_InPos()*/{
			fsm_state = SERVO_RUNTER;
		}
		break;

	// Lademechanismus nach unten fahren
	case SERVO_RUNTER:

		//Evtl. Os_delay() Funktion verwenden damit Würfel nicht mit Lichtgeschwindigkeit aufgelanden wird?
		servoCtr++;
		if (servoCtr>=100){
			servoCtr=0;
			servoAngle = Servo_GetAngle(); // Winkel auslesen
			Servo_SetAngle(servoAngle++); // Winkel vergrössern
			if (servoAngle >= 90){
				fsm_state = SERVO_RAUF;
			}
		}
		break;

	// Lademechanismus nach oben fahren
	case SERVO_RAUF:
		servoCtr++;
		if (servoCtr>=100){
			servoCtr=0;
			servoAngle = Servo_GetAngle(); // Winkel auslesen
			Servo_SetAngle(servoAngle--); // Winkel verkleinern
			if (servoAngle <= 0){
				fsm_state = WURFEL_ZURUCKFAHREN;
			}
		}
		break;

	// Würfelsensor zur Erkennposition zurückfahren -> Kontrolle Würfel geladen
	case WURFEL_ZURUCKFAHREN:
		PID_Pos(posWurfel-DistTofToWurfel); // Zurückfahren zur Würfelerkennposition

		if(wurfel_erkennen()==TASK_OK){
			wurfelCtr++;
			if (wurfelCtr >= MaxLoadAttempts){ // maximale Anzahl Versuche erreicht
				wurfelCtr=0;
				fsm_state = STARTPOSITION;
			}
			else{
				posWurfel = Quad_GetPos()-WurfelLength;
				fsm_state = WURFEL_VORFAHREN;
			}
		}
		else{ // TimeOut -> Würfel ist aufgeladen
			wurfelCtr=0;
			fsm_state = STARTPOSITION;
		}
		break;

	// Startposition anfahren -> Anlauf holen
	case STARTPOSITION:
		PID_Pos(posStart); // Startposition anfahren

		// TODO implement method PID_InPos in pid.c
		if (0) /*PID_InPos()*/{
			fsm_state = SCHNELLFAHRT;
		}
		break;

	// Schnellfahrt (Beschleunigen und Abbruchkriterien checken)
	case SCHNELLFAHRT:
		accCtr++;
		if (accCtr>=100){
			speed++;
			if (speed >= MaxVelo){
				speed = MaxVelo;
			}
			PID_Velo(speed);
		}

		// Anzahl Runden erreicht
		// TODO implement method getNbrRounds in Raspi.c
		if (0) /*(getNbrRounds >= MaxNbrRounds)*/{
			accCtr = 0;
			fsm_state = BREMSEN;
		}
		 // Zu viele Signale erkannt
		// TODO implement method getNbrSignals in Raspi.c
		else if (0) /*(getNbrSignals() >= MaxNbrSignals)*/{
			accCtr = 0;
			fsm_state = BREMSEN;
		}
		// Gemessene Streck grösser als maximale Streckenlänge
		else if ((Quad_GetPos()-posStart)>=MaxTrackLength){
			accCtr = 0;
			fsm_state = BREMSEN;
		}
		break;

	// Abbremsen zur Haltesignalerkennung
	case BREMSEN:
		accCtr++;
		if (accCtr>=100){
			speed--;
			if (speed <= SlowVelo){
				fsm_state = FINALES_HALTESIGNAL;
			}
			PID_Velo(speed);
		}
		break;

	// Warten auf Signal finales Haltesignal erkannt von Raspi
	case FINALES_HALTESIGNAL:
		PID_Velo(SlowVelo);
		// TODO implement method getHaltesignal in Raspi.c
		if (0)/*getHaltesignal()*/{ // finales Haltesignal erkannt
			fsm_state = HALTESIGNAL_ANFAHREN;
		}

		break;

	// Haltesignal mit TOF erkennen
	case HALTESIGNAL_ANFAHREN:
		PID_Velo(SlowVelo);
		if(haltesignal_erkennen()==TASK_OK){
			posHaltesignal=Quad_GetPos();
			distHaltesignal=getDistanceValue();
			fsm_state = HALTESIGNAL_STOPPEN;
		}
		else{
			//Was machen wir wenn das Haltesignal nicht erkannt wird???
		}
		break;

	// Positionregelung vor Haltesignal
	case HALTESIGNAL_STOPPEN:
		PID_Pos(posHaltesignal+distHaltesignal);
		// TODO implement method PID_InPos in pid.c
		if (0) /*PID_InPos()*/{
			fsm_state = STOP;
		}
		break;

	// Warten bis Startsignal von Raspi zurückgenommen wird
	case STOP:
		if (0) /*!getStartSignal()*/{
			fsm_state = STARTUP;
		}
		break;
	}
    osDelay(50);
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
  //uint8_t txData[3];
  //char x = 'x';
  //char y = 'y';
  //char z = 'z';
  uint8_t testInt;
  /* Infinite loop */
  for(;;)
  {
	  //Im Main.c gibt es bei der Initialiserung eine Funktion, die Entscheidet, ob dieser Task ausgeführt wird oder nicht
	  if(getEnableSensorTask() == 1){
		  /*if(measureAccel3AxisValues()==TASK_OK){
			  testInt = getZValue();
		  }*/
		  if(measureDistanceValue()==TASK_OK){
				testInt = getDistanceValue();
				if(testInt <60){
					__NOP();
				}
		  }
		  //txData[0] = (uint8_t) z;
		  //txData[1] = (uint8_t) (getZValue() >> 8);
		  //txData[2] = (uint8_t) (getZValue() & 0xff);

		  //txData[0] = getDistanceValue();
		  //HAL_UART_Transmit(&huart2, txData, 3, 100);
		  osDelay(50);
	  }

	  else{
		  osDelay(5000);
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

	uint8_t enableTask = 0;
  /* Infinite loop */
  for(;;)
  {

	  if(enableTask == 1){

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
	  			  if(PID_GetEnable()==1){
	  				  PID_Velo((-1)*getfinalVelocity());
	  				  PID_SetEnable(0);
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
	  			if(PID_GetEnable()==1){
	  				  PID_Velo(getfinalVelocity());
	  				  PID_SetEnable(0);
	  			  }
	  		  }

	  		}


	  	//Imlement a Error Handler witch sets de "drehrichtung" value to 0, if we dont receive Interrupts from UART1
	  	//--> Not implemented yet
	  	else{
	  		 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
	  		 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);

	  	 }

	  	  osDelay(10);


	  }
	  else{
		  osDelay(5000);
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

	uint8_t enableTask = 0;
  uint16_t servoPWM = 0;

  for(;;)
  {
	  if(enableTask ==1){
		  servoPWM = 26400 + getSpeedGroupValue()*2740;
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, servoPWM);
		  osDelay(50);
	  }

	  else{
		  osDelay(5000);
	  }

  }
  /* USER CODE END StartTask04 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
