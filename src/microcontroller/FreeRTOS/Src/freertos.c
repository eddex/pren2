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
#include "stm32f3xx_hal.h"
#include <motor_v.h>
#include <motor_h.h>
#include <pid_v.h>
#include <pid_h.h>
#include <quad_v.h>
#include <quad_h.h>
#include <velocity_v.h>
#include <velocity_h.h>
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
#include "servo.h"
#include "fsm.h"
#include "DataTransfer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SlowVeloWurfel 150 // langsame Geschwindigkeit für Würfelaufnahme [mm/s]
#define SlowVeloHaltesignal 150 // langsame Geschwindigkeit für Haltesignal [mm/s]
#define MaxVelo 3000 // maximale Geschwindigkeit [mm/s]
#define MaxNbrSignals 10 // maximale Anzahl Signale auf der Strecke
#define MaxNbrRounds 3 // maximale Anzahl Runden
#define MaxLoadAttempts 2 // maximale Anzahl Würfelladeversuche
#define MaxTrackLength 25000 // maximale Streckenlänge [mm]
#define OffsetStartpos 1000 // Offset beim Retourfahren zur Startposition zur Verhinderung überfahren der Startposition [Ticks]
#define OffsetHaltesignal 80 // Offset Halten beim Haltesignal [mm]
#define OffsetWurfel 20 // Offset Vorfahren beim Würfel [mm]

//fsm State wurde von Default Task hierher verschoben, damit im sendDataToRaspy Task darauf zugegriffen werden kann
enum fsm fsm_state; // create enum for statemachine task

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId fsm_tskHandle;
osThreadId tof_tskHandle;
osThreadId radio_tskHandle;
osThreadId uart_tskHandle;
osThreadId acc_tskHandle;
osSemaphoreId myBinarySem01Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void FSM_Task(void const * argument);
void TOF_Task(void const * argument);
void Radio_Task(void const * argument);
void UART_Task(void const * argument);
void Accel_Task(void const * argument);

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
  /* definition and creation of fsm_tsk */
  osThreadDef(fsm_tsk, FSM_Task, osPriorityNormal, 0, 128);
  fsm_tskHandle = osThreadCreate(osThread(fsm_tsk), NULL);

  /* definition and creation of tof_tsk */
  osThreadDef(tof_tsk, TOF_Task, osPriorityNormal, 0, 128);
  tof_tskHandle = osThreadCreate(osThread(tof_tsk), NULL);

  /* definition and creation of radio_tsk */
  osThreadDef(radio_tsk, Radio_Task, osPriorityNormal, 0, 128);
  radio_tskHandle = osThreadCreate(osThread(radio_tsk), NULL);

  /* definition and creation of uart_tsk */
  osThreadDef(uart_tsk, UART_Task, osPriorityNormal, 0, 128);
  uart_tskHandle = osThreadCreate(osThread(uart_tsk), NULL);

  /* definition and creation of acc_tsk */
  osThreadDef(acc_tsk, Accel_Task, osPriorityNormal, 0, 128);
  acc_tskHandle = osThreadCreate(osThread(acc_tsk), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_FSM_Task */
/**
  * @brief  Function implementing the fsm_tsk thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_FSM_Task */
void FSM_Task(void const * argument)
{

  /* USER CODE BEGIN FSM_Task */

	fsm_state = STARTUP; // Default State -> Startup
	uint32_t fsm_state_test;

	taskState_t taskState;
	taskState = TASK_RUNNING;

	uint8_t wurfelCtr = 0; // Zähler Anzahl Ladeversuche

	int32_t posStart = 0;	// Positionsmerker bei Startsignal
	int32_t posFinalesHaltesignal = 0; // Positionsmerker bei warten auf finales Haltesignal
	int32_t posWurfel = 0;	// Positionsmerker bei Würfel erkannt
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
				posStart=Quad_V_GetPos();
				startTimeMeasurment();			//Zeitmessung beginnen für Abbruchkriterium des Tasks

				HAL_GPIO_WritePin(LED_Heartbeat_GPIO_Port, LED_Heartbeat_Pin, GPIO_PIN_RESET);
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

			HAL_GPIO_WritePin(LED_Heartbeat_GPIO_Port, LED_Heartbeat_Pin, GPIO_PIN_RESET);
			fsm_state = WURFEL_ERKENNEN;
			#endif
			break;

		// Warten bis Würfel erkennt wird
		case WURFEL_ERKENNEN:
			PID_V_Velo(SlowVeloWurfel); //Mit langsamer Geschwindigkeit fahren
			PID_H_Velo(SlowVeloWurfel);

			taskState = tof_erkennen(130); //Versuche den Wüfel in einer Distanz bis zu 13cm zu erkennen

			if(taskState == TASK_OK){
				HAL_GPIO_WritePin(LED_Heartbeat_GPIO_Port, LED_Heartbeat_Pin, GPIO_PIN_SET);
				//fsm_state = SERVO_RUNTER;
				posWurfel=Quad_V_GetPos() + ((OffsetWurfel * iGetriebe * TicksPerRev) / (Wirkumfang)); // Umrechnung Millimeter in Ticks
				fsm_state = WURFEL_VORFAHREN;
			}
			else if(taskState == TASK_TIME_OVERFLOW){ // Würfel nicht erkannt
				fsm_state = STARTPOSITION;
			}
			else{}

			break;

		// Vorfahren bis zur Würfelmitte
		case WURFEL_VORFAHREN:
			PID_V_Velo(SlowVeloWurfel); //Mit langsamer Geschwindigkeit fahren
			PID_H_Velo(SlowVeloWurfel);

			if (Quad_V_GetPos()>=posWurfel){
				Motor_V_Break();
				Motor_H_Break();
				fsm_state = SERVO_RUNTER;
			}
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
			PID_V_Velo(-SlowVeloWurfel); // Mit langsamer Geschwindigkeit an die ursprüngliche Position zurück fahren
			PID_H_Velo(-SlowVeloWurfel);

			//Wenn die ursprüngliche Position beim zurückfahren erreicht wurde
			if ((Quad_V_GetPos()<(posStart+OffsetStartpos) && taskState==TASK_OK)||(Quad_V_GetPos()<(posStart+OffsetStartpos) && tryAgain ==1)){	//Konnte der Wüfel nach max. 3 Versuchen geladen werden?
				Motor_V_Break();
				Motor_H_Break();
				suspendSensorTask=1; //Task suspended
				osDelay(10);
				HAL_GPIO_WritePin(GPIOF, SHDN_TOF_KLOTZ_Pin, GPIO_PIN_RESET); // Tof Klotz disable
				fsm_state = SCHNELLFAHRT;
			}
			else if((Quad_V_GetPos()<(posStart+OffsetStartpos) && tryAgain==0)){	//Würfel konnte nach 3 Versuchen nicht geladen werden
				//Versuche noch einmal den Würfel zu erkennen
				tryAgain = 1;
				Motor_V_Break();
				Motor_H_Break();
				startTimeMeasurment();
				fsm_state = WURFEL_ERKENNEN;
			}
			break;

		// Schnellfahrt (Beschleunigen und Abbruchkriterien checken)
		case SCHNELLFAHRT:
			//Geschwindigkeitsrampe bis max.Speed
			speed+=10;
			if (speed >= MaxVelo){
				speed = MaxVelo;
			}
			PID_V_Velo(speed);
			PID_H_Velo(speed);

			// Anzahl Runden erreicht
			if (getFlagStructure().roundCounter >= MaxNbrRounds){
				fsm_state = BREMSEN;
			}
			 // Zu viele Signale erkannt
			else if (getFlagStructure().signalCounter >= MaxNbrSignals){
				fsm_state = BREMSEN;
			}
			// Gemessene Strecke grösser als maximale Streckenlänge
			else if ((Quad_V_GetPos()-posStart)>=((MaxTrackLength * iGetriebe * TicksPerRev) / (Wirkumfang))){
				fsm_state = BREMSEN;
			}
			break;

		// Abbremsen zur Haltesignalerkennung
		case BREMSEN:
			speed-=10;
			if (speed <= SlowVeloHaltesignal){
				posFinalesHaltesignal = Quad_V_GetPos(); // Position merken
				fsm_state = FINALES_HALTESIGNAL;
			}
			PID_V_Velo(speed);
			PID_H_Velo(speed);
			break;

		// Warten auf Signal finales Haltesignal erkannt von Raspi
		case FINALES_HALTESIGNAL:
			PID_V_Velo(SlowVeloHaltesignal);
			PID_H_Velo(SlowVeloHaltesignal);
			//Annahme, dass finales Haltesignal nie erkannt wird --> Bsp. Signal Counter bleit konstant...
			//Abbruchbedingung für Schwenken der weissen Flagge definieren...
			//ToDo

			if ((getFlagStructure().finalHSerkannt) /*|| (getFlagStructure().roundCounter<MaxNbrRounds)*/){ // finales Haltesignal erkannt oder Raspberry abgekratzt
				HAL_GPIO_WritePin(GPIOF, SHDN_TOF_TAFEL_Pin, GPIO_PIN_SET); // Tof Tafel enable
				resetDistanceValue(); // Reset Distance Value for next measurement
				VL6180X_Init();
				suspendSensorTask=0; //Enable Sensor Task
				startTimeMeasurment();
				fsm_state = HALTESIGNAL_ANFAHREN;
				PID_V_ClearError();
				PID_H_ClearError();
			}
			// falls eine ganze Strecke abgefahren wurde und kein Haltesignal von Raspberry erhalten wurde
			/*
			else if	((Quad_V_GetPos()-posFinalesHaltesignal)>=(((MaxTrackLength/2) * iGetriebe * TicksPerRev) / (Wirkumfang))){
				HAL_GPIO_WritePin(GPIOF, SHDN_TOF_TAFEL_Pin, GPIO_PIN_SET); // Tof Tafel enable
				resetDistanceValue(); // Reset Distance Value for next measurement
				VL6180X_Init();
				suspendSensorTask=0; //Enable Sensor Task
				startTimeMeasurment();
				fsm_state = HALTESIGNAL_ANFAHREN;
				PID_V_ClearError();
				PID_H_ClearError();
			}*/

			/*
			HAL_GPIO_WritePin(GPIOF, SHDN_TOF_TAFEL_Pin, GPIO_PIN_SET); // Tof Tafel enable
			resetDistanceValue(); // Reset Distance Value for next measurement
			VL6180X_Init();
			suspendSensorTask=0; //Enable Sensor Task
			startTimeMeasurment();
			fsm_state = HALTESIGNAL_ANFAHREN;
			PID_V_ClearError();
			PID_H_ClearError();
			*/

			#if WuerfelerkenneUndLaden_TEST
				HAL_GPIO_WritePin(GPIOF, SHDN_TOF_TAFEL_Pin, GPIO_PIN_SET); // Tof Tafel enable
				resetDistanceValue(); // Reset Distance Value for next measurement
				VL6180X_Init();
				suspendSensorTask=0; //Enable Sensor Task
				startTimeMeasurment();
				fsm_state = HALTESIGNAL_ANFAHREN;
				PID_V_ClearError();
				PID_H_ClearError();
			#endif
			break;

		// Haltesignal mit TOF erkennen
		case HALTESIGNAL_ANFAHREN:
			PID_V_Velo(SlowVeloHaltesignal);
			PID_H_Velo(SlowVeloHaltesignal);
			taskState=tof_erkennen(100);

			if(taskState==TASK_OK){
				HAL_GPIO_WritePin(GPIOF, SHDN_TOF_TAFEL_Pin, GPIO_PIN_RESET); // Tof Tafel disable
				distHaltesignal=(((getDistanceValue()-OffsetHaltesignal) * iGetriebe * TicksPerRev) / (Wirkumfang)); // Umrechnung Millimeter in Ticks
				posHaltesignal=Quad_V_GetPos()+distHaltesignal; // Speicherung Position Haltesignal
				fsm_state = HALTESIGNAL_STOPPEN;
			}
			else if(taskState==TASK_TIME_OVERFLOW){
				//Was machen wir wenn das Haltesignal nicht erkannt wird???
				HAL_GPIO_WritePin(GPIOF, SHDN_TOF_TAFEL_Pin, GPIO_PIN_RESET); // Tof Tafel disable
				posHaltesignal=Quad_V_GetPos(); // Speicherung Position Haltesignal
				fsm_state = HALTESIGNAL_STOPPEN;
			}
			else{
				//Task is Running
			}
			break;

		// Positionregelung vor Haltesignal
		case HALTESIGNAL_STOPPEN:
			PID_V_Velo(SlowVeloHaltesignal);
			PID_H_Velo(SlowVeloHaltesignal);
			if (Quad_V_GetPos()>=posHaltesignal){
				Motor_V_Break();
				Motor_H_Break();
				HAL_GPIO_WritePin(HB_Sleep_GPIO_Port, HB_Sleep_Pin, GPIO_PIN_RESET); // Disable H-Bridge
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
			fsm_state = fsm_state;

			break;
		}
		fsm_state_test=fsm_state;
		osDelay(10);

	#else
	  osDelay(9000);
	#endif
	}

  /* USER CODE END FSM_Task */
}

/* USER CODE BEGIN Header_TOF_Task */
/**
* @brief Function implementing the tof_tsk thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TOF_Task */
void TOF_Task(void const * argument)
{
  /* USER CODE BEGIN TOF_Task */

	/* Infinite loop */
	for(;;)
	{
		#if SensorTaskEnable
		if(suspendSensorTask==0){
			if(measureDistanceValue()==TASK_OK){}
			osDelay(10);
		}

		else{
			//Error Handling ist still ToDo
			osDelay(10);
		}
		#else
			osDelay(9000);
		#endif
	}
  /* USER CODE END TOF_Task */
}

/* USER CODE BEGIN Header_Radio_Task */
/**
* @brief Function implementing the radio_tsk thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Radio_Task */
void Radio_Task(void const * argument)
{
  /* USER CODE BEGIN Radio_Task */
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
				Motor_V_Break();
				Motor_H_Break();
				osDelay(1000);
			}
			else{													//Drive with UART1 received pwmValue
				if(PID_V_GetEnable()==1){
					PID_V_Velo((-1)*getfinalVelocity());
					PID_H_Velo((-1)*getfinalVelocity());
					PID_V_SetEnable(0);
					PID_H_SetEnable(0);
					}
				}
			}

		//Drehrichtung REVERSE
		else if(getDrehrichtung() == 114){
			firstForwardCount = 0;

			//Break function
			if(firstReverseCount==0){
				firstReverseCount++;
				Motor_V_Break();
				Motor_H_Break();
				osDelay(1000);
			}
			else{												//Drive with UART1 received pwmVlue
				//Nur alle 10ms!
				if(PID_V_GetEnable()==1){
					PID_V_Velo(getfinalVelocity());
					PID_H_Velo(getfinalVelocity());
					PID_V_SetEnable(0);
					PID_H_SetEnable(0);
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
		#else
	  	osDelay(9000);
		#endif
	}
  /* USER CODE END Radio_Task */
}

/* USER CODE BEGIN Header_UART_Task */
/**
* @brief Function implementing the uart_tsk thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART_Task */
void UART_Task(void const * argument)
{
  /* USER CODE BEGIN UART_Task */
	/* Infinite loop */

	uint8_t UartSendBuffer[1];
	UartSendBuffer[0] = 0x21;	//Opcode of UART Spec: Respond all Sensor Data

	uint16_t X_Accel_buffer=0;
	uint16_t Y_Accel_buffer=0;
	uint16_t Z_Accel_buffer=0;
	uint8_t changeSendByte=0;

	for(;;)
	{
		#if UARTSendRaspyData
		switch(changeSendByte){
			case 0:
				UartSendBuffer[0] = 0x21;	//Opcode
				changeSendByte++;
				break;
			case 1:
				UartSendBuffer[0] = 0;		//TOF 1: Fail, its not possible to read out both Distance Sensors...
				changeSendByte++;
				break;
			case 2:
				UartSendBuffer[0] = getDistanceValue();	//Actual TOF
				changeSendByte++;
				break;
			case 3:
				X_Accel_buffer = getXValue();
				UartSendBuffer[0] = (uint8_t) (X_Accel_buffer >> 8);		//X_Accel_high
				changeSendByte++;
				break;
			case 4:
				UartSendBuffer[0] = (uint8_t) (X_Accel_buffer & 0xff);		//X_Accel_low
				changeSendByte++;
				break;
			case 5:
				Y_Accel_buffer = getYValue();
				UartSendBuffer[0] = (uint8_t) (Y_Accel_buffer >> 8);		//Y_Accel_high
				changeSendByte++;
				break;
			case 6:
				UartSendBuffer[0] = (uint8_t) (Y_Accel_buffer & 0xff);		//Y_Accel_low
				changeSendByte++;
				break;
			//Acc. to Interface Spec. UART should case 7 be Speed High Data
			case 7:
				Z_Accel_buffer = getZValue();
				UartSendBuffer[0] = (uint8_t) (Z_Accel_buffer >> 8);		//Z_Accel_high
				changeSendByte++;
				break;
			//Acc. to Interface Spec. UART should case 7 be Speed Low Data
			case 8:
				UartSendBuffer[0] = (uint8_t) (Z_Accel_buffer & 0xff);		//Z_Accel_low
				changeSendByte++;
				break;
			case 9:
				UartSendBuffer[0] = Servo_GetAngle();		//Actual Servo Angle
				changeSendByte++;
				break;
			case 10:
				UartSendBuffer[0] = fsm_state;				//Actual FSM State
				changeSendByte=0;
				break;
		}
		#if !SensorTaskEnable
			UartSendBuffer[3] = 0;
			UartSendBuffer[4] = 0;
			UartSendBuffer[5] = 0;
			UartSendBuffer[6] = 0;
			UartSendBuffer[7] = 0;
			UartSendBuffer[8] = 0;
		#endif

		HAL_UART_Transmit(&huart1, UartSendBuffer, 1, 1000);

		//*********Virtual Comport UART Debug**************
		HAL_UART_Transmit(&huart2, UartSendBuffer, 1, 1000);
		//*************************************************
		osDelay(200);
		#else
		osDelay(9000);
		#endif
	}
  /* USER CODE END UART_Task */
}

/* USER CODE BEGIN Header_Accel_Task */
/**
* @brief Function implementing the acc_tsk thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Accel_Task */
void Accel_Task(void const * argument)
{
  /* USER CODE BEGIN Accel_Task */
	/* Infinite loop */
	for(;;)
	{
#if SensorTaskEnable
		if(measureAccel3AxisValues()==TASK_OK){
			//testInt = getZValue();
		}
		osDelay(10);
#else
		osDelay(9000);
#endif
  }
  /* USER CODE END Accel_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
