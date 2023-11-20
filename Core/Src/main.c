/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WATCHDOG_INTERVAL_MS 5000
volatile bool watchdog_expired = false;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
ADC_ChannelConfTypeDef sConfig;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void start_watchdog_timer(void);
void stop_watchdog_timer(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  char msg[40];
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  // Values of each potentiometer
  long unsigned int adc_value_ch0 = 0;
  long unsigned int adc_value_ch1 = 0;
  long unsigned int adc_value_ch2 = 0;
  long unsigned int adc_value_ch3 = 0;
  long unsigned int adc_value_ch4 = 0;

  // Switching between each reading
  int adc_channel = 0;

  // Is the user's posture different, time since the last change in posture, last and current posture
  bool diff_posture = false;
  long unsigned int time_since_posture_change = 0;
  long unsigned int last_posture[5], current_posture[5] = {0, 0, 0, 0, 0};

  // Initial reading to allow logic to begin with full data
  bool initial_read = false;

  // Conversion constant for ADC to degrees
  long unsigned int adc_deg_conv_const = 11.655555555555;
  long unsigned int testtimer = 0;
  start_watchdog_timer();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if (watchdog_expired) {
	        // Handle the watchdog timeout
	        // ...

	        // Reset the watchdog timer
	        stop_watchdog_timer();
	        start_watchdog_timer();
	  }

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
	// Make readings of the potentiometers using ADC, only reads one at a time to prevent erroneous values
    if (adc_channel == 0){ // Reads the first potentiometer (A0)
    	// Sets the analog pin to read (A0)
    	sConfig.Channel = ADC_CHANNEL_0;
    	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    	// Reads the pin (A0)
    	HAL_ADC_Start(&hadc1);
    	HAL_ADC_PollForConversion(&hadc1, 100);
    	adc_value_ch0 = HAL_ADC_GetValue(&hadc1);

    	sprintf(msg, "ADC Channel 0 value: %lu\r\n", adc_value_ch0); // Debug purposes

    	adc_channel++; // Moves to the next potentiometer to read

    } else if (adc_channel == 1){ // Reads the second potentiometer (A1)
    	// Sets the analog pin to read (A1)
    	sConfig.Channel = ADC_CHANNEL_1;
    	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    	// Reads the pin (A1)
    	HAL_ADC_Start(&hadc1);
    	HAL_ADC_PollForConversion(&hadc1, 100);
    	adc_value_ch1 = HAL_ADC_GetValue(&hadc1);

    	sprintf(msg, "ADC Channel 1 value: %lu\r\n", adc_value_ch1); // Debug purposes

    	adc_channel++; // Moves to the next potentiometer to read

    } else if (adc_channel == 2){
    	// Sets the analog pin to read (A2)
    	sConfig.Channel = ADC_CHANNEL_4;
    	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    	// Reads the pin (A2)
    	HAL_ADC_Start(&hadc1);
    	HAL_ADC_PollForConversion(&hadc1, 100);
    	adc_value_ch2 = HAL_ADC_GetValue(&hadc1);

    	sprintf(msg, "ADC Channel 2 value: %lu\r\n", adc_value_ch2); // Debug purposes

    	adc_channel++; // Moves to the next potentiometer to read

    } else if (adc_channel == 3){
    	// Sets the analog pin to read (A3)
    	sConfig.Channel = ADC_CHANNEL_8;
    	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    	// Reads the pin (A3)
    	HAL_ADC_Start(&hadc1);
    	HAL_ADC_PollForConversion(&hadc1, 100);
    	adc_value_ch3 = HAL_ADC_GetValue(&hadc1);

    	sprintf(msg, "ADC Channel 3 value: %lu\r\n", adc_value_ch3); // Debug purposes

    	adc_channel++; // Moves to the next potentiometer to read

    } else if (adc_channel == 4){
    	// Sets the analog pin to read (A4)
    	sConfig.Channel = ADC_CHANNEL_11;
    	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    	// Reads the pin (A4)
    	HAL_ADC_Start(&hadc1);
    	HAL_ADC_PollForConversion(&hadc1, 100);
    	adc_value_ch4 = HAL_ADC_GetValue(&hadc1);

    	sprintf(msg, "ADC Channel 4 value: %lu\r\n", adc_value_ch4); // Debug purposes

    	if (initial_read == false) { // Makes the initial reading which allows the logic to start
    		long unsigned int last_posture[5] = {adc_value_ch0, adc_value_ch1, adc_value_ch2, adc_value_ch3, adc_value_ch4};
    		initial_read = true;

    	}

    	// Sets the current posture for the logic
    	long unsigned int current_posture[5] = {adc_value_ch0, adc_value_ch1, adc_value_ch2, adc_value_ch3, adc_value_ch4};

    	adc_channel = 0; // Resets back to reading the first potentiometer

    } else {
    	adc_channel = 0; // Catches any error and resets to the first potentiometer

    }

    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY); // Convert to string and print (debug purposes

    if (initial_read){ // When the device gathers the initial data necessary, begin logic
        if (((current_posture[0] + current_posture[1] + current_posture[2] + current_posture[3] + current_posture[4]) - (last_posture[0] + last_posture[1] + last_posture[2] + last_posture[3] + last_posture[4])) / adc_deg_conv_const > (10 + 0.28)){ // Is the user's posture within 10 degrees + 100 Ohms error
        	diff_posture = true; // Detects a different posture in the user
        }

        if (diff_posture == true && HAL_GetTick() - time_since_posture_change > 600000){ // If the user has a different posture and it has not been 10 minutes, remind them to keep their current posture, otherwise update their posture and reset the timer
        	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
        	time_since_posture_change = HAL_GetTick();
        	long unsigned int last_posture[5] = {current_posture[0], current_posture[1], current_posture[2], current_posture[3], current_posture[4]};
        	diff_posture == false;
        } else if (diff_posture == true) { // Remind the user to keep their posture
        	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
        }

        if (HAL_GetTick() - time_since_posture_change > 1800000){ // If it has been 30 minutes since the last change in posture, pulse every 1.5 seconds to remind them to change it
        	if ((HAL_GetTick() - time_since_posture_change - 1800000) % 3000 < 1500){
        		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
        	} else {
        		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
        	}
        } else if (diff_posture == false){ // Turn off the device if their posture is as it should be
        	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
        }
    }


    // Test code
    if (HAL_GetTick() - testtimer > 10000){
    	testtimer = HAL_GetTick();
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    	HAL_Delay(1000);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    }



    HAL_Delay(100); // Delay to allow smooth operation

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void start_watchdog_timer(void)
{
  HAL_Delay(WATCHDOG_INTERVAL_MS);
  watchdog_expired = true;
}

void stop_watchdog_timer(void)
{
  watchdog_expired = false;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
