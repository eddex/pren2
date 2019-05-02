
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "RadioModule.h"
#include "AccelSens_MMA8451.h"
#include "DistSens_VL6180X.h"
#include "velocity.h"
#include "quad.h"
#include "motor.h"
#include "pid.h"
#include "servo.h"
#include "DataTransfer.h"
#include "SEGGER/SEGGER_SYSVIEW.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

//*************TIMER variable declaration******************************************
//#define Timer3MaxCounterPeriod 500	//Variable mit der Timerperiode für die Berechnung des PWMs --> CubeMX Value
uint8_t tim15Count10ms = 0;				//Variable die im TIM15 overflow incrementiert wird und für eine 10ms Zeitbasis verwendet wird

//*************UART variable declaration******************************************

//Receive Data from Radio Module
uint8_t rx_dataUART1_RadioModule[]={0,0,0,0};	//Receive Data Buffer for Radio Moudl data
uint8_t receivedSpeedValue = 0;		//Calculated Speedvalue Group with rx_dataUART1_RadioModule
uint16_t pwmValue = 0;				//(not used) Finally used pwmValue for the pwm generation

//Receive DAta from Boardcomputer
uint8_t rx_dataUART1_Boardcomputer[]={0,0};
uint8_t storeFlagValue = 0;
uint8_t storeNextByte = 0;


//debugVariable
uint8_t tx_dataUART2[] = {0,0,0,0};		//Transmit Data Buffer for UART2 Debugging

// function before main to initialise
/*
void __attribute__ ((constructor)) premain()
{
	HAL_GPIO_WritePin(HB_Sleep_GPIO_Port, HB_Sleep_Pin, GPIO_PIN_RESET);
}*/

//******************************************************************************************
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	HAL_GPIO_WritePin(HB_Sleep_GPIO_Port, HB_Sleep_Pin, GPIO_PIN_RESET);
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */


  /* ***************H-Bridge Control************************
   * IN1 = 0 & IN2 = PWM 	--> Reverse
   * IN2 = 0 & IN1 = PWM 	--> Forward
   * IN1 = 1 & IN2 = 1		--> Brake
   *********************************************************/

  /*
   *Motor Regelung
   */
  Quad_Init();
  Velo_Init();
  PID_Init();
  Motor_Init();
  Servo_Init();

  //If Sensortask enabled
#if SensorTaskEnable
  //Enable only the Tof for laoding the Wood-Klotz ;-)

  //This Config is now in FSM Startup Task
  //HAL_Delay(10);
  //HAL_GPIO_WritePin(GPIOF, SHDN_TOF_KLOTZ_Pin, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(GPIOF, SHDN_TOF_TAFEL_Pin, GPIO_PIN_RESET);
  //VL6180X_Init();											//Init of VL6180X Distance Sensor Device

  if(MMA8451_Init()==TASK_OK){											//Init of MMA8451 Accel Sensor Device
	  HAL_GPIO_WritePin(LED_Heartbeat_GPIO_Port, LED_Heartbeat_Pin, GPIO_PIN_RESET);
  }
  else{
	  HAL_GPIO_WritePin(LED_Heartbeat_GPIO_Port, LED_Heartbeat_Pin, GPIO_PIN_SET);
  }
#endif

  /*
   * *****************UART1 Configuration*************************
   * UART1 RX @PA10 => D0		-		TX Pin of ER400TRS-02
   * UART1 TX @PA9 => D1		-		RX Pin of ER400TRS-02
   *
   * Baud rate 19200 Bit/s
   * 8 Data Bit
   * No Parity
   * 1 Stop Bit
   */
  //Interrupt enable for UART1 Receive Function "HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)"
#if FunkFernsteuer_BoardcomputerBetrieb
  HAL_UART_Receive_IT(&huart1, rx_dataUART1_RadioModule, 1);
#else
  HAL_UART_Receive_IT(&huart1, rx_dataUART1_Boardcomputer, 1);
#endif

  //***************************************************************


  /*
   * ****************Timer2 Configuration*************************
   * CHANNEL 2 @ PA1 => A1	-		Signal Wire of ServoMotor
   *
   * Clk_Source = 24MHz
   * Counter Period is 480'000 Counts
   * PWM Frequency = Clk_source/Counts = 24'000'000/480'000 = 50Hz
   *
   * This PWM Output is used for Servomotor control
   * Servo Angle-Control:
   * -90°	=> 1.1ms (PWM_Value = 11 * (480'000Counts / 200) = 26'400
   * 0°		=> 1.5ms (PWM_Value = 15 * (480'000Counts / 200) = 36'000
   * 90°	=> 1.9ms (PWM_Value = 19 * (480'000Counts / 200) = 45'600
   *
   * Interrupt Enabled
   */
  //timeBase Start for Interrupt
  HAL_TIM_Base_Start_IT(&htim2);

  //CHannel Enable
  HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);

  //Channel Compare Value --> PWM Dutycycle
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 26400);				//Servomotor Default Position 0°
  //****************************************************************


  /*
   * ****************Timer3 Configuration**************************
   * CHANNEL 1 @ PB4 => D12		-		IN1	HBridge MotorV
   * CHANNEL 2 @ PA4 => A3		-		IN2	HBridge MotorV
   * CHANNEL 3 @ PB0 => D3		-		IN1 HBridge MotorH
   * CHANNEL 4 @ PB1 => D6		-		IN2	HBridge MotorH
   *
   *
   * Encoder MotorH
   * Enc_ChA_MOT_H @ PA0 	=> A0
   * Enc_ChB_MOT_H @ PA12 	=> D2
   *
   * * Encoder MotorV
   * Enc_ChA_MOT_V @ PA7 	=> A6
   * Enc_ChB_MOT_V @ PA3 	=> A2
   *
   * Clk_Source = 24MHz
   * Counter Period is 20'000 Counts --> Change in Header #define Timer3MaxCounterPeriod
   * PWM Frequency = Clk_source/Counts = 24'000'000/20'000 = 1200Hz
   *
   * Interrupt Enabled with PWM Frequency of 1200Hz
   */
  //TimeBase Start for Interrupt
  HAL_TIM_Base_Start_IT(&htim3);

  //CHannel Enable
  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_4);

  //Channel Compare Value --> PWM Dutycycle
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
  //****************************************************************



  //****************Timer15 Configuration**************************
  //				    Interrupt ONLY
  //TimeBase Start for Interrupt
  HAL_TIM_Base_Start_IT(&htim15);
  //********************************************************************

  SEGGER_SYSVIEW_Conf(); // Start SystemViewer

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* USER CODE END 2 */

  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_HSI;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USER CODE BEGIN 4 */
/*
 * Function is called after a complete Transmition of UART Data
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

#if FunkFernsteuer_BoardcomputerBetrieb

	HAL_UART_Receive_IT(&huart1, rx_dataUART1_RadioModule, 1);						//Restart Interrupt reception mode

	if(rx_dataUART1_RadioModule[0]>47 && rx_dataUART1_RadioModule[0]<56){			//rx_dataUART1_RadioModule with the received values between 48 an 55
		receivedSpeedValue = rx_dataUART1_RadioModule[0]-48;						//Speed group 0 - 7
		setSpeedGroupValue(receivedSpeedValue);										//Store Speedgroupvalue, used in ServoMotorTask
		//pwmValue = (uint16_t)((Timer3MaxCounterPeriod/7)*receivedSpeedValue);		//Final PWM Value from 0(0% duty cycle) to 20000(100% duty cycle);
	}

	//Über Funkmodul erhaltener Wert ist kein Geschwindigkeitswert sonder für die Drehrichtung
	else {
		setDrehrichtung(rx_dataUART1_RadioModule[0]);								//Valid Values: 108 /114
	}

	//Depending on Speedgroup (0-7) calculate the Motorvelocity in u/s
	if(receivedSpeedValue == 0){
		setfinalVelocity(0);
	}
	else if (receivedSpeedValue == 1){
		setfinalVelocity(100);
	}
	else if (receivedSpeedValue == 2){
		setfinalVelocity(300);
	}
	else if (receivedSpeedValue == 3){
		setfinalVelocity(500);
	}
	else if (receivedSpeedValue == 4){
		setfinalVelocity(1000);
	}
	else if (receivedSpeedValue == 5){
		setfinalVelocity(1500);
	}
	else if (receivedSpeedValue == 6){
		setfinalVelocity(2000);
	}
	else{
		setfinalVelocity(3000);
	}


	//Debug via UART2 Virtual Comport-------------
	//tx_dataUART2[0] = rx_dataUART1[0];//rx_dataUART1[0];
	//HAL_UART_Transmit(&huart2, tx_dataUART2, 1, 1000);
	//----------------------------------------------

	//First Steps were with UART2 via Virtual Comport
	//HAL_UART_Receive_IT(&huart2, rx_dataUART1, 1);				//restart Interrupt reception mode
	//HAL_UART_Transmit(&huart2, tx_data, 1, 1000);

#else
	//Real Code
	HAL_UART_Receive_IT(&huart1, rx_dataUART1_Boardcomputer, 1);	//Restart Interrupt reception mode

	//If one Byte before was the Sync Character
	if(storeNextByte == 1){
		setFlagStructure(rx_dataUART1_Boardcomputer[0]);			//Store the essential Data

		tx_dataUART2[0] = getFlagStructure().startSignal;
		tx_dataUART2[1] = getFlagStructure().finalHSerkannt;
		tx_dataUART2[2] = getFlagStructure().roundCounter;
		tx_dataUART2[3] = getFlagStructure().signalCounter;

		HAL_UART_Transmit(&huart2, tx_dataUART2, 4, 1000);

		//----------------------------------------------
	}

	if(rx_dataUART1_Boardcomputer[0]==0x7f && storeNextByte == 0){	//Sync-Caracter detected
		storeNextByte = 1;
	}
	else{
		storeNextByte = 0;
	}

#endif
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  else if(htim->Instance == TIM15){
  		tim15Count10ms++;											//Variable to create a 10ms Timebase

  		//Every 10ms
  		if(tim15Count10ms == 200){
  			Velo_Sample();
  			tim15Count10ms=0;
  			PID_SetEnable(1);										//Enables the PID funciton in freertos.c --> MotorControl

  			incrementTimeMeasurmentValue();							//Variable for Timemeasurement
  		}
  		//Every 50us
  		Quad_Sample();
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
