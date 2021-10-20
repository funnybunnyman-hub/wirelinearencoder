/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "stdio.h"
//#include "string.h"
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

/* USER CODE BEGIN PV */

//for communication
uint8_t tx_buffer[BUFFER_SIZE_SPI3] = {0x0a, 0x0b, 0x1a, 0x1b, 0x2a, 0x2b, 0x3a, 0x3b};
uint8_t rx_buffer[BUFFER_SIZE_SPI3] = {READ_CONFIG,0,0x00,0x00,0,0x00,0x00,0x00};
uint8_t buffer_updated = 0;
Controller_State slave_state = READ_CONFIG;

//for wire encoder
int WireLength[ENCODER_NUMBER] = {0};
int WireLength_mm[ENCODER_NUMBER] = {0};
unsigned char pointer_ENC[ENCODER_NUMBER] = {0};
uint8_t readpin[ENCODER_NUMBER] = {0};
static struct Encoder_SIGNAL_Info ENC_infos[ENCODER_NUMBER] = {{WE_0A_Pin, WE_0B_Pin, WE_0A_GPIO_Port, WE_0B_GPIO_Port}, {WE_1A_Pin, WE_1B_Pin, WE_1A_GPIO_Port, WE_1B_GPIO_Port}, {WE_2A_Pin, WE_2B_Pin, WE_2A_GPIO_Port, WE_2B_GPIO_Port}};
int count = 0; int count2 = 0;

//for air pressure sensor
//uint32_t adc_val[ADC_NUMBER];
uint16_t adc_buffer[ADC_NUMBER];
int i;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//static inline void copy_buffer(uint8_t dest_buffer[], uint8_t dest_size, uint8_t src_buffer[], uint8_t src_size) {
//  for (uint8_t idx = 0; idx < src_size; idx++) {
//    dest_buffer[idx] = src_buffer[idx];
//  }
//}

static inline void set_state(uint8_t command_idx) {
  switch (command_idx) {
    case 0xFD: //11111101, 253
      slave_state = WIREENCODER;
      break;

    case 0xFC: //11111100, 252
          slave_state = AIRPRESSURE;
          break;

    case 0xFB: //11111011, 252
      slave_state = CHECKING_SPI;
      break;

    default:
      slave_state = READ_CONFIG;
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == SPI3) {
//		tx_buffer[7] = slave_state;
		HAL_SPI_TransmitReceive_DMA(&hspi3, tx_buffer, rx_buffer, BUFFER_SIZE_SPI3);
        buffer_updated = 1;
//        tx_buffer[0] = rx_buffer[0];
	}
	set_state(rx_buffer[0]);
	count++;
//	static short Led = 1;
//	if(count >= 10000){
//		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, Led);
//		Led = 1 - Led;
//		count = 0;
//	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim6){
		int d[16]={0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; //1:cw, -1:ccw, 0:none
		for(i = 0; i < ENCODER_NUMBER; i++){
			readpin[i] = (HAL_GPIO_ReadPin(ENC_infos[i].A_port, ENC_infos[i].A_pin) << 1) | HAL_GPIO_ReadPin(ENC_infos[i].B_port, ENC_infos[i].B_pin);
			pointer_ENC[i] = (pointer_ENC[i] <<2 | readpin[i]) & 0x0f;
			WireLength[i] += d[pointer_ENC[i]];
		}
	}
//	static short Led = 1;
//	if(count2 >= 10000){
//		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, Led);
//		Led = 1 - Led;
//		count2 = 0;
//	}
//	count2++;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
  //After AD Convert
}

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
  MX_DMA_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);

  /* Configure Regular Channel */
//  sConfig.Channel = ADC_CHANNEL_1;
//  sConfig.Rank = 1;
//  sConfig.SingleDiff = ADC_SINGLE_ENDED;
//  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
//  sConfig.OffsetNumber = ADC_OFFSET_NONE;
//  sConfig.Offset = 0;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sConfig.Channel = ADC_CHANNEL_2;
//  sConfig.Rank = 2;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sConfig.Channel = ADC_CHANNEL_3;
//  sConfig.Rank = 3;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_SPI_TransmitReceive_DMA(&hspi3, tx_buffer, rx_buffer, BUFFER_SIZE_SPI3);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_NUMBER);
  while (1)
  {
	  count2++;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  switch(slave_state) {
		  case READ_CONFIG:
//	  			  copy_buffer(tx_buffer, BUFFER_SIZE_SPI1, rx_buffer, BUFFER_SIZE_SPI1);
			  if (buffer_updated == 1) {
				  buffer_updated = 0;
			  }
			  break;

		  case CHECKING_SPI:
//	  			  copy_buffer(tx_buffer, BUFFER_SIZE_SPI1, rx_buffer, BUFFER_SIZE_SPI1);
//	  			  tx_buffer[0] = rx_buffer[0];
			  if (buffer_updated == 1) {
				  buffer_updated = 0;
  //				  tx_buffer[1] = 0x00;
  //				  tx_buffer[2] = 0x00;
  //				  tx_buffer[3] = 0x00;
  //				  tx_buffer[4] = 0x00;
  //				  tx_buffer[5] = 0x00;
  //				  tx_buffer[6] = 0x00;
				  tx_buffer[1] = 0xfc;
				  tx_buffer[2] = 0xfc;
				  tx_buffer[3] = 0xfc;
				  tx_buffer[4] = 0xfc;
				  tx_buffer[5] = 0xfc;
				  tx_buffer[6] = 0xfc;
			  }
			  break;
		  case WIREENCODER:
			  if (buffer_updated == 1) {
//	  				copy_buffer(tx_buffer, BUFFER_SIZE_SPI1, rx_buffer, BUFFER_SIZE_SPI1);
				//tx_buffer[0] = rx_buffer[0];
  //				  HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
  //				  uint8_t L1 = 0; uint8_t L2 = 0;
				  for(i = 0; i < ENCODER_NUMBER; i++){
					  if(WireLength[0] >= 0){
						  tx_buffer[2*i] = WireLength[i] >> 8;
						  tx_buffer[2*i+1] = WireLength[i];
					  }
					  else{
						  tx_buffer[2*i] = 0;
						  tx_buffer[2*i+1] = 0;
					  }
				  }
				  buffer_updated = 0;
			  }
			  break;
		  case AIRPRESSURE:
//	  			  copy_buffer(tx_buffer, BUFFER_SIZE_SPI1, rx_buffer, BUFFER_SIZE_SPI1);
			  for(i = 0; i < ADC_NUMBER; i++){
				  tx_buffer[2*i] = (uint8_t)(adc_buffer[i] >> 8);
				  tx_buffer[2*i+1] = (uint8_t)adc_buffer[i];
			  }
			  if (buffer_updated == 1) {
				  buffer_updated = 0;
			  }
			  break;
	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
