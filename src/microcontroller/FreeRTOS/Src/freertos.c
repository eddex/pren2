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
#define MaxVelo 3000 // maximale Geschwindigkeit [mm/s]
#define MaxNbrSignals 10 // maximale Anzahl Signale auf der Strecke
#define MaxNbrRounds 2 // maximale Anzahl Runden
#define MaxLoadAttempts 4 // maximale Anzahl Würfelladeversuche
#define MaxTrackLength 20000 // maximale Streckenlänge [mm]

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
uint8_t suspendSensorTask = 1; 		//0: Task is Running   / 1: Task suspended
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
  int32_t posHaltesignal = 0; // Positionsmerker Haltesignal erkannt
  int32_t distHaltesignal = 0; // Distanz zum Haltesignal

  uint32_t servoCtr = 0; // Zähler um Servo langsam zu bewegen

  uint8_t servoAngle = 0; // Winkelmerker Servo
  uint16_t speed = 0; // Geschwindigkeit der Motoren

  uint8_t tryAgain = 0; 	//Würfel konnte nach 3 mal nicht geladen werden--> Das ganze noch mal Versuchen

  /* Infinite loop */
  for(;;)
  {

#if FSMTaskEnable

	//Statemachine acc. to PREN1 Documentation p.24
	switch(fsm_state){
	// Warten auf Startbefehl von Raspi
	case STARTUP:
		//getStartSignal is available with the function_call "flags_UartData_t getFlagStructure(void)" see for more info in usart.h
		if (getFlagStructure().startSignal){
			HAL_GPIO_WritePin(GPIOF, SHDN_TOF_KLOTZ_Pin, GPIO_PIN_SET); // Tof Klotz enable
			HAL_GPIO_WritePin(GPIOF, SHDN_TOF_TAFEL_Pin, GPIO_PIN_RESET); // Tof Tafel disable
			resetDistanceValue(); // Reset Distance Value for next measurement
			VL6180X_Init();
			suspendSensorTask=0; //Enable Sensor Task
			startTimeMeasurment();

			//Enable H-Bridge Module of Motor1 and Motor2
			HAL_GPIO_WritePin(HB_Sleep_GPIO_Port, HB_Sleep_Pin, GPIO_PIN_SET);
			posStart=Quad_GetPos();
			startTimeMeasurment();			//Zeitmessung beginnen für Abbruchkriterium des Tasks
			fsm_state = WURFEL_ERKENNEN;
		}

		#if WuerfelerkenneUndLaden_TEST
		HAL_GPIO_WritePin(GPIOF, SHDN_TOF_KLOTZ_Pin, GPIO_PIN_SET); // Tof Klotz enable
		HAL_GPIO_WritePin(GPIOF, SHDN_TOF_TAFEL_Pin, GPIO_PIN_RESET); // Tof Tafel disable
		resetDistanceValue(); // Reset Distance Value for next measurement
		VL6180X_Init();
		suspendSensorTask=0; //Enable Sensor Task
		startTimeMeasurment();

		//Enable H-Bridge Module of Motor1 and Motor2
		HAL_GPIO_WritePin(HB_Sleep_GPIO_Port, HB_Sleep_Pin, GPIO_PIN_SET);
		startTimeMeasurment();					//Zeitmessung beginnen für Abbruchkriterium des Tasks
		fsm_state = WURFEL_ERKENNEN;
		#endif
		break;

	// Warten bis Würfel erkennt wird
	case WURFEL_ERKENNEN:
		PID_Velo(SlowVelo); //Mit langsamer Geschwindigkeit fahren

		taskState = tof_erkennen(130); //Versuche den Wüfel in einer Distanz bis zu 13cm zu erkennen

		if(taskState == TASK_OK){
			fsm_state = SERVO_RUNTER;
		}
		else if(taskState == TASK_TIME_OVERFLOW){ // Würfel nicht erkannt
			fsm_state = STARTPOSITION;
		}
		else{}

		break;

	// Lademechanismus nach unten fahren
	// Der Winkel wird alle 50ms um 1° erhöht.
	case SERVO_RUNTER:
		servoCtr++;
		if (servoCtr>=5){
			servoCtr=0;
			servoAngle = Servo_GetAngle(); 	// Winkel auslesen
			Servo_SetAngle(++servoAngle); 	// Winkel vergrössern
			if (servoAngle >= 90){
				fsm_state = SERVO_RAUF;
			}
		}
		break;

	// Lademechanismus nach oben fahren
	// Der Winkel wird alle 50ms um 1° verringert.
	case SERVO_RAUF:
		servoCtr++;
		if (servoCtr>=5){
			servoCtr=0;
			servoAngle = Servo_GetAngle(); // Winkel auslesen
			Servo_SetAngle(--servoAngle); // Winkel verkleinern
			if (servoAngle <= 0){
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);		//Entlastet PWM von Servomotor, damit am Anschlag nicht weitergefahren wird.
				startTimeMeasurment();
				fsm_state = WURFEL_KONTROLLE;
			}
		}
		break;

	// Kontrolle Würfel geladen
	// Falls er erfolgreich geladen wurde, wird die Startposition angefahren
	// Falls nicht geladen, werden weitere Ladeversuche gestartet
	case WURFEL_KONTROLLE:
		taskState=tof_erkennen(40);
		if(taskState==TASK_OK){  // Wenn Objekt in einem Abstand von bis zu 4cm erkannt wurde --> Würfel geladen
			wurfelCtr=0;
			fsm_state = STARTPOSITION;
		}
		else{
			wurfelCtr++;
			if (wurfelCtr >= MaxLoadAttempts){ // maximale Anzahl Versuche erreicht
				wurfelCtr=0;
				fsm_state = STARTPOSITION;
			}
			else{
				//Versuche erneut den Würfel zu greifen
				fsm_state = SERVO_RUNTER;
			}
		}
		break;

	// Startposition anfahren -> Anlauf holen
	case STARTPOSITION:


		PID_Velo(-SlowVelo); // Mit langsamer Geschwindigkeit an die ursprüngliche Position zurück fahren

		//Wenn die ursprüngliche Position beim zurückfahren erreicht wurde
		if ((Quad_GetPos()<posStart && taskState==TASK_OK)||(Quad_GetPos()<posStart && tryAgain ==1)){	//Konnte der Wüfel nach max. 3 Versuchen geladen werden?
			Motor_Break();
			suspendSensorTask=1; //Task suspended
			osDelay(10);
			HAL_GPIO_WritePin(GPIOF, SHDN_TOF_KLOTZ_Pin, GPIO_PIN_RESET); // Tof Klotz disable
			fsm_state = SCHNELLFAHRT;
		}
		else if((Quad_GetPos()<posStart && tryAgain==0)){	//Würfel konnte nach 3 Versuchen nicht geladen werden
			//Versuche noch einmal den Würfel zu erkennen
			tryAgain = 1;
			Motor_Break();
			startTimeMeasurment();
			fsm_state = WURFEL_ERKENNEN;
		}



		/*if (PID_InPos()){
			fsm_state = SCHNELLFAHRT;
		}*/
		break;

	// Schnellfahrt (Beschleunigen und Abbruchkriterien checken)
	case SCHNELLFAHRT:
		//Geschwindigkeitsrampe bis max.Speed
		speed+=10;
		if (speed >= MaxVelo){
			speed = MaxVelo;
		}
		PID_Velo(speed);

		// Anzahl Runden erreicht
		if (getFlagStructure().roundCounter >= MaxNbrRounds){
			fsm_state = BREMSEN;
		}
		 // Zu viele Signale erkannt
		else if (getFlagStructure().signalCounter >= MaxNbrSignals){
			fsm_state = BREMSEN;
		}
		// Gemessene Strecke grösser als maximale Streckenlänge
		else if ((Quad_GetPos()-posStart)>=((MaxTrackLength * iGetriebe * TicksPerRev) / (Wirkumfang))){
			fsm_state = BREMSEN;
		}
		break;

	// Abbremsen zur Haltesignalerkennung
	case BREMSEN:
		speed-=10;
		if (speed <= SlowVelo){
			fsm_state = FINALES_HALTESIGNAL;
		}
		PID_Velo(speed);
		break;

	// Warten auf Signal finales Haltesignal erkannt von Raspi
	case FINALES_HALTESIGNAL:
		PID_Velo(SlowVelo);
		//Annahme, dass finales Haltesignal nie erkannt wird --> Bsp. Signal Counter bleit konstant...
		//Abbruchbedingung für Schwenken der weissen Flagge definieren...
		//ToDo

		if (getFlagStructure().finalHSerkannt){ // finales Haltesignal erkannt
			HAL_GPIO_WritePin(GPIOF, SHDN_TOF_TAFEL_Pin, GPIO_PIN_SET); // Tof Tafel enable
			resetDistanceValue(); // Reset Distance Value for next measurement
			VL6180X_Init();
			suspendSensorTask=0; //Enable Sensor Task
			startTimeMeasurment();
			fsm_state = HALTESIGNAL_ANFAHREN;
			PID_ClearError();

		}

		#if WuerfelerkenneUndLaden_TEST
			HAL_GPIO_WritePin(GPIOF, SHDN_TOF_TAFEL_Pin, GPIO_PIN_SET); // Tof Tafel enable
			resetDistanceValue(); // Reset Distance Value for next measurement
			VL6180X_Init();
			suspendSensorTask=0; //Enable Sensor Task
			startTimeMeasurment();
			fsm_state = HALTESIGNAL_ANFAHREN;
			PID_ClearError();
		#endif
		break;

	// Haltesignal mit TOF erkennen
	case HALTESIGNAL_ANFAHREN:
		PID_Velo(SlowVelo);
		taskState=tof_erkennen(100);

		if(taskState==TASK_OK){
			HAL_GPIO_WritePin(GPIOF, SHDN_TOF_TAFEL_Pin, GPIO_PIN_RESET); // Tof Tafel disable

			/*
			#if WuerfelerkenneUndLaden_TEST
				Motor_Break();
				HAL_GPIO_WritePin(HB_Sleep_GPIO_Port, HB_Sleep_Pin, GPIO_PIN_RESET); // disable H-Bridge
				while(1);
			#endif*/

			distHaltesignal=((getDistanceValue() * iGetriebe * TicksPerRev) / (Wirkumfang)); // Umrechnung Millimeter in Ticks
			posHaltesignal=Quad_GetPos()+distHaltesignal; // Speicherung Position Haltesignal
			fsm_state = HALTESIGNAL_STOPPEN;
		}
		else if(taskState==TASK_TIME_OVERFLOW){
			//Was machen wir wenn das Haltesignal nicht erkannt wird???
			#if WuerfelerkenneUndLaden_TEST
				Motor_Break();
				HAL_GPIO_WritePin(HB_Sleep_GPIO_Port, HB_Sleep_Pin, GPIO_PIN_RESET);
				while(1);
			#endif
		}
		else{
			//Task is Running
		}
		break;

	// Positionregelung vor Haltesignal
	case HALTESIGNAL_STOPPEN:
		PID_Velo(SlowVelo);
		if (Quad_GetPos()>=posHaltesignal){
			Motor_Break();
			fsm_state = STOP;
		}
		/*
		PID_Pos(posHaltesignal);
		if (PID_InPos()){
			fsm_state = STOP;
		}*/
		break;

	// Warten bis Startsignal von Raspi zurückgenommen wird
	case STOP:
		/*
		if (!(getFlagStructure().startSignal)){	//!getStartSignal() Was meinst du mit dem Andi?
			fsm_state = STARTUP;
		}*/

		break;
	}
    osDelay(10);

#else
  osDelay(9000);
#endif
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

  /* Infinite loop */
  for(;;)
  {
	#if SensorTaskEnable

	  if(measureAccel3AxisValues()==TASK_OK){
		//testInt = getZValue();
	  }


	  if(suspendSensorTask==0){
		  if(measureDistanceValue()==TASK_OK){}
		  osDelay(200);
	  }

	  else{
		//Error Handling ist still ToDo
		  osDelay(200);
	  }

	#else
		  osDelay(9000);
	#endif
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

	//Allow H_Bridge to Control the Motors
#if FunkFernsteuer_BoardcomputerBetrieb
	HAL_GPIO_WritePin(HB_Sleep_GPIO_Port, HB_Sleep_Pin, GPIO_PIN_SET);
#endif


  /* Infinite loop */
  for(;;)
  {
	#if FunkFernsteuer_BoardcomputerBetrieb

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
	#else
  		osDelay(9000);
	}
	#endif

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

  uint8_t UartSendBuffer[11];
  UartSendBuffer[0] = 0x21;	//Opcode of UART Spec: Respond all Sensor Data

  uint16_t X_Accel_buffer=0;
  uint16_t Y_Accel_buffer=0;
  uint16_t Z_Accel_buffer=0;

  for(;;)
  {
	#if UARTSendRaspyData
	  UartSendBuffer[1] = 0;									//TOF 1: Fail, its not possible to read out both Distance Sensors...
	  UartSendBuffer[2] = 0;//getDistanceValue();					//TOF 2

	  #if SensorTaskEnable//!!!!!!!!
	  //X_Accel_buffer = getXValue();
	  UartSendBuffer[3] = 0;//(uint8_t) (X_Accel_buffer >> 8);		//X_Accel_high
	  UartSendBuffer[4] = 0;//(uint8_t) (X_Accel_buffer & 0xff);	//X_Accel_low

	  //Y_Accel_buffer = getYValue();
	  UartSendBuffer[5] = 0;//(uint8_t) (Y_Accel_buffer >> 8);		//Y_Accel_high
	  UartSendBuffer[6] = 0;//(uint8_t) (Y_Accel_buffer & 0xff);	//Y_Accel_low

	  //Z_Accel_buffer = getZValue();
	  UartSendBuffer[7] = 0;	//(uint8_t) (Z_Accel_buffer >> 8);		//Z_Accel_high
	  UartSendBuffer[8] = 0;	//(uint8_t) (Z_Accel_buffer & 0xff);	//Z_Accel_low


	  #else
	  UartSendBuffer[3] = -1;
	  UartSendBuffer[4] = -1;
	  UartSendBuffer[5] = -1;
	  UartSendBuffer[6] = -1;
	  UartSendBuffer[7] = -1;
	  UartSendBuffer[8] = -1;
	  #endif

	  UartSendBuffer[9] = 0;//Servo_GetAngle();		//Servo Data
	  UartSendBuffer[10] = 0;		//FSM State

	  HAL_UART_Transmit(&huart1, UartSendBuffer, 11, 1000);


	  //*********Virtual Comport UART Debug**************
	  //HAL_UART_Transmit(&huart2, UartSendBuffer, 11, 1000);
	  //*************************************************

	  osDelay(500);

	  //Implementation to Do

	#else
	osDelay(9000);

	#endif

  }
  /* USER CODE END StartTask04 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
