
/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_ADC_CHANNELS 1
/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/
uint16_t channels[16]= {
	       1000, 900, 1200, 900,
	       1500, 1500, 1500, 1500,
	       1500, 1500, 1500, 1500,
	      1500, 1500, 1500, 1500
	    };
// Change if using more channels
//uint16_t channels[NUM_ADC_CHANNELS];
ADC_HandleTypeDef hadc1;
RTC_HandleTypeDef hrtc;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
/* USER CODE BEGIN PV */
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
// Add this before `main()`:
//void readJoystickChannels(void);
void readJoystickChannels(ADC_HandleTypeDef *hadc, uint8_t adcIndex);
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SBUS_GenerateFrame(uint16_t channels[16], uint8_t sbus_data[25]) {
	 memset(sbus_data, 0, 25);
	sbus_data[0] = 0x0F;
 uint32_t ch[16];
 sbus_data[1] = (uint8_t)(ch[0] & 0xFF);
 sbus_data[2] = (uint8_t)((ch[0] >> 8) | (ch[1] << 3));
 sbus_data[3] = (uint8_t)((ch[1] >> 5) | (ch[2] << 6));
 sbus_data[4] = (uint8_t)(ch[2] >> 2);
 sbus_data[5] = (uint8_t)((ch[2] >> 10) | (ch[3] << 1));
 sbus_data[6] = (uint8_t)((ch[3] >> 7) | (ch[4] << 4));
 sbus_data[7] = (uint8_t)((ch[4] >> 4) | (ch[5] << 7));
 sbus_data[8] = (uint8_t)(ch[5] >> 1);
 sbus_data[9] = (uint8_t)((ch[5] >> 9) | (ch[6] << 2));
 sbus_data[10] = (uint8_t)((ch[6] >> 6) | (ch[7] << 5));
 sbus_data[11] = (uint8_t)(ch[7] >> 3);
 sbus_data[12] = (uint8_t)(ch[8] & 0xFF);
 sbus_data[13] = (uint8_t)((ch[8] >> 8) | (ch[9] << 3));
 sbus_data[14] = (uint8_t)((ch[9] >> 5) | (ch[10] << 6));
 sbus_data[15] = (uint8_t)(ch[10] >> 2);
 sbus_data[16] = (uint8_t)((ch[10] >> 10) | (ch[11] << 1));
 sbus_data[17] = (uint8_t)((ch[11] >> 7) | (ch[12] << 4));
 sbus_data[18] = (uint8_t)((ch[12] >> 4) | (ch[13] << 7));
 sbus_data[19] = (uint8_t)(ch[13] >> 1);
 sbus_data[20] = (uint8_t)((ch[13] >> 9) | (ch[14] << 2));
 sbus_data[21] = (uint8_t)((ch[14] >> 6) | (ch[15] << 5));
 sbus_data[22] = (uint8_t)(ch[15] >> 3);
 sbus_data[23] = 0x00; // flags
 sbus_data[24] = 0x00; // end byte

}
void SBUS_Transmit(UART_HandleTypeDef *huart, uint16_t channels[16]) {
 uint8_t sbus_data[25];
 SBUS_GenerateFrame(channels, sbus_data);
 HAL_UART_Transmit(huart, sbus_data, 25, HAL_MAX_DELAY);
}
/* USER CODE END 0 */
/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
 /* USER CODE BEGIN 1 */
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
 MX_RTC_Init();
 MX_USART1_UART_Init();
 MX_USART2_UART_Init();
 MX_ADC1_Init();
 /* USER CODE BEGIN 2 */
//  HAL_GPIO_WritePin(ls_bat_GPIO_Port, ls_bat_Pin, SET);
//  	HAL_Delay(100);
//  	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
//  	HAL_ADC_Start(&hadc1);
//  	HAL_ADC_PollForConversion(&hadc1, 20);
//  	float adc_val = 0;
 /* USER CODE END 2 */
 /* Infinite loop */
 /* USER CODE BEGIN WHILE */
 while (1)
 {
//	  adc_val = HAL_ADC_GetValue(&hadc1);
//	  channels[3]=adc_val;
//	  readJoystickChannels();
	  // Read from ADC1 and assign to channels[0] to channels[3]
	          readJoystickChannels(&hadc1, 3);
	          // Read from ADC2 and assign to channels[4] to channels[7]
//	          readJoystickChannels(&hadc2, 4);
	  SBUS_Transmit(&huart2, channels);
	  	  	  	  HAL_Delay(70);
//        if (adc_val==0)
//        	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
//        else
//        	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
//	 	  printf("%f\n\r\n", adc_val);
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
 RCC_OscInitTypeDef RCC_OscInitStruct = {0};
 RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
 RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
 /** Initializes the RCC Oscillators according to the specified parameters
 * in the RCC_OscInitTypeDef structure.
 */
 RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
 RCC_OscInitStruct.HSIState = RCC_HSI_ON;
 RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
 RCC_OscInitStruct.LSIState = RCC_LSI_ON;
 RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
 RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
 RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
 RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
 if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
 {
   Error_Handler();
 }
 /** Initializes the CPU, AHB and APB buses clocks
 */
 RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                             |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
 RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
 RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
 RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
 RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
 if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
 {
   Error_Handler();
 }
 PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                             |RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC12;
 PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
 PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
 PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
 PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
 if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
 ADC_MultiModeTypeDef multimode = {0};
 ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
 ADC_ChannelConfTypeDef sConfig = {0};
 /* USER CODE BEGIN ADC1_Init 1 */
 /* USER CODE END ADC1_Init 1 */
 /** Common config
 */
 hadc1.Instance = ADC1;
 hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
 hadc1.Init.Resolution = ADC_RESOLUTION_12B;
 hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
 hadc1.Init.ContinuousConvMode = ENABLE;
 hadc1.Init.DiscontinuousConvMode = DISABLE;
 hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
 hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
 hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
 hadc1.Init.NbrOfConversion = 1;
 hadc1.Init.DMAContinuousRequests = DISABLE;
 hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
 hadc1.Init.LowPowerAutoWait = DISABLE;
 hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
 if (HAL_ADC_Init(&hadc1) != HAL_OK)
 {
   Error_Handler();
 }
 /** Configure the ADC multi-mode
 */
 multimode.Mode = ADC_MODE_INDEPENDENT;
 if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
 {
   Error_Handler();
 }
 /** Configure Analog WatchDog 1
 */
 AnalogWDGConfig.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
 AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
 AnalogWDGConfig.HighThreshold = 0;
 AnalogWDGConfig.LowThreshold = 0;
 AnalogWDGConfig.Channel = ADC_CHANNEL_1;
 AnalogWDGConfig.ITMode = DISABLE;
 if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
 {
   Error_Handler();
 }
 /** Configure Regular Channel
 */
 sConfig.Channel = ADC_CHANNEL_1;
 sConfig.Rank = ADC_REGULAR_RANK_1;
 sConfig.SingleDiff = ADC_SINGLE_ENDED;
 sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
 sConfig.OffsetNumber = ADC_OFFSET_NONE;
 sConfig.Offset = 0;
 if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
 {
   Error_Handler();
 }
 /* USER CODE BEGIN ADC1_Init 2 */
 /* USER CODE END ADC1_Init 2 */
}
/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void)
{
 /* USER CODE BEGIN RTC_Init 0 */
 /* USER CODE END RTC_Init 0 */
 /* USER CODE BEGIN RTC_Init 1 */
 /* USER CODE END RTC_Init 1 */
 /** Initialize RTC Only
 */
 hrtc.Instance = RTC;
 hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
 hrtc.Init.AsynchPrediv = 127;
 hrtc.Init.SynchPrediv = 255;
 hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
 hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
 hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
 if (HAL_RTC_Init(&hrtc) != HAL_OK)
 {
   Error_Handler();
 }
 /* USER CODE BEGIN RTC_Init 2 */
 /* USER CODE END RTC_Init 2 */
}
/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{
 /* USER CODE BEGIN USART1_Init 0 */
 /* USER CODE END USART1_Init 0 */
 /* USER CODE BEGIN USART1_Init 1 */
 /* USER CODE END USART1_Init 1 */
 huart1.Instance = USART1;
 huart1.Init.BaudRate = 38400;
 huart1.Init.WordLength = UART_WORDLENGTH_8B;
 huart1.Init.StopBits = UART_STOPBITS_1;
 huart1.Init.Parity = UART_PARITY_NONE;
 huart1.Init.Mode = UART_MODE_TX_RX;
 huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
 huart1.Init.OverSampling = UART_OVERSAMPLING_16;
 huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
 huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
 if (HAL_UART_Init(&huart1) != HAL_OK)
 {
   Error_Handler();
 }
 /* USER CODE BEGIN USART1_Init 2 */
 /* USER CODE END USART1_Init 2 */
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
 huart2.Init.BaudRate = 100000;
 huart2.Init.WordLength = UART_WORDLENGTH_8B;
 huart2.Init.StopBits = UART_STOPBITS_2;
 huart2.Init.Parity = UART_PARITY_EVEN;
 huart2.Init.Mode = UART_MODE_TX_RX;
 huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
 huart2.Init.OverSampling = UART_OVERSAMPLING_16;
 huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
 huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_TXINVERT_INIT;
 huart2.AdvancedInit.TxPinLevelInvert = UART_ADVFEATURE_TXINV_ENABLE;
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
 __HAL_RCC_GPIOA_CLK_ENABLE();
 __HAL_RCC_GPIOB_CLK_ENABLE();
 __HAL_RCC_GPIOF_CLK_ENABLE();
 /*Configure GPIO pin Output Level */
 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
 /*Configure GPIO pin : PB0 */
 GPIO_InitStruct.Pin = GPIO_PIN_0;
 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
 /*Configure GPIO pin : PF14 */
 GPIO_InitStruct.Pin = GPIO_PIN_14;
 GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
 /* USER CODE BEGIN MX_GPIO_Init_2 */
 /* USER CODE END MX_GPIO_Init_2 */
}
/* USER CODE BEGIN 4 */
//uint16_t mapJoystickADCValue(uint16_t adc_val, uint16_t min_val, uint16_t max_val)
//{
//    return ((adc_val * (max_val - min_val)) / 4095) + min_val;
//}
void readJoystickChannels(ADC_HandleTypeDef *hadc, uint8_t adcIndex)
{
   uint32_t adc_val;
   for (uint8_t i = 0; i < NUM_ADC_CHANNELS; i++)
   {
       HAL_ADC_Start(hadc);
       if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK)
       {
           adc_val = HAL_ADC_GetValue(hadc);
           // Map to 1000-2000 range
           channels[adcIndex + i] =          (adc_val * 1000 / 4095) + 1000;
       }
       HAL_ADC_Stop(hadc);
   }
}
//void readJoystickChannels()
//{
//    HAL_ADC_Start(&hadc1);
//
//    for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
//        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//        uint16_t raw_val = HAL_ADC_GetValue(&hadc1);
//        channels[i] = mapJoystickADCValue(raw_val, 1000, 2000);
//    }
//
//    HAL_ADC_Stop(&hadc1);
//}
// int _write(int file, char *ptr, int len)
//{
//  (void)file;
//  int DataIdx;
//
//  for (DataIdx = 0; DataIdx < len; DataIdx++)
//  {
//	  ITM_SendChar(*ptr++);
////    __io_putchar(*ptr++);
//  }
//  return len;
//}
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


