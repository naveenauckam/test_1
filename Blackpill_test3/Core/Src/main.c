/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "MCP4725.h"
#include <stdbool.h>

#define DAC8051_add_w  0x90
#define DAC8051_add_r  0x91

uint8_t tx_data1 = 0x08 ;
uint16_t rx_data1 = 0 ;


//uint8_t tx_data2 = 0x08

uint16_t tx_data2 = 0 ;
uint16_t tx_data3 = 65500 ;






uint16_t parallel_data_temp = 0;
uint16_t parallel_data = 0;
 uint16_t serial_data_temp = 0;
 uint16_t serial_data_temp1 = 0;
 uint16_t serial_data= 0;
bool is_mcp_connected = 0;
bool  input0 = false ;
 bool temp = false ;

MCP4725 myMCP4725;
uint32_t micro_seconds = 0;

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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void parallel_data_read()
{
	parallel_data_temp = 0x0000;
	parallel_data_temp = HAL_GPIO_ReadPin(Input_0_GPIO_Port, Input_0_Pin) | 0x0000 ;
    parallel_data_temp |= HAL_GPIO_ReadPin(Input_1_GPIO_Port, Input_1_Pin) << 1 ;
    parallel_data_temp |= HAL_GPIO_ReadPin(Input_2_GPIO_Port, Input_2_Pin) << 2 ;
    parallel_data_temp |= HAL_GPIO_ReadPin(Input_3_GPIO_Port, Input_3_Pin) << 3 ;
    parallel_data_temp |= HAL_GPIO_ReadPin(Input_4_GPIO_Port, Input_4_Pin) << 4 ;
    parallel_data_temp |= HAL_GPIO_ReadPin(Input_5_GPIO_Port, Input_5_Pin) << 5 ;
    parallel_data_temp |= HAL_GPIO_ReadPin(Input_6_GPIO_Port, Input_6_Pin) << 6 ;
    parallel_data_temp |= HAL_GPIO_ReadPin(Input_7_GPIO_Port, Input_7_Pin) << 7 ;
    parallel_data_temp |= HAL_GPIO_ReadPin(Input_8_GPIO_Port, Input_8_Pin) << 8 ;
    parallel_data_temp |= HAL_GPIO_ReadPin(Input_9_GPIO_Port, Input_9_Pin) << 9 ;
    parallel_data_temp |= HAL_GPIO_ReadPin(Input_10_GPIO_Port, Input_10_Pin) << 10 ;
    parallel_data_temp |= HAL_GPIO_ReadPin(Input_11_GPIO_Port, Input_11_Pin) << 11 ;
    parallel_data_temp |= HAL_GPIO_ReadPin(Input_12_GPIO_Port, Input_12_Pin) << 12 ;
    parallel_data_temp |= HAL_GPIO_ReadPin(Input_13_GPIO_Port, Input_13_Pin) << 13 ;
    parallel_data_temp |= HAL_GPIO_ReadPin(Input_14_GPIO_Port, Input_14_Pin) << 14 ;
    parallel_data_temp |= HAL_GPIO_ReadPin(Input_15_GPIO_Port, Input_15_Pin) << 15 ;

    parallel_data_temp = ~ parallel_data_temp ;
    parallel_data = parallel_data_temp ;

}

void serial_data_read()
{
	HAL_GPIO_WritePin(EN_1_GPIO_Port , EN_1_Pin , 0);
	HAL_GPIO_WritePin(EN_2_GPIO_Port , EN_2_Pin , 0);
	HAL_GPIO_WritePin(PL_1_GPIO_Port , PL_1_Pin , 0);
	HAL_Delay(2);
	HAL_GPIO_WritePin(PL_1_GPIO_Port , PL_1_Pin , 1);
	HAL_GPIO_WritePin(PL_2_GPIO_Port , PL_2_Pin , 1);


    serial_data_temp = 0;
	serial_data_temp1 = 0;

	HAL_Delay(10);


//	serial_data_temp =  HAL_GPIO_ReadPin(Shift_data_out_GPIO_Port, Shift_data_out_Pin);


     	for(int i=0 ; i<7 ; i++)
		{
			HAL_GPIO_WritePin(CLK_GPIO_Port , CLK_Pin , 1);
		    HAL_Delay(2);
		    HAL_GPIO_WritePin(CLK_GPIO_Port , CLK_Pin , 0);
			HAL_Delay(2);
		}



	for(int i=0 ; i<8 ; i++)
	{
		HAL_GPIO_WritePin(CLK_GPIO_Port , CLK_Pin , 1);
	    HAL_Delay(2);
	    HAL_GPIO_WritePin(CLK_GPIO_Port , CLK_Pin , 0);
		HAL_Delay(2);
		temp = HAL_GPIO_ReadPin(Shift_data_out_GPIO_Port, Shift_data_out_Pin) ;
		serial_data_temp |=  HAL_GPIO_ReadPin(Shift_data_out_GPIO_Port, Shift_data_out_Pin) << i;
	}

	HAL_GPIO_WritePin(EN_1_GPIO_Port , EN_1_Pin , 1);
	HAL_GPIO_WritePin(PL_2_GPIO_Port , PL_2_Pin , 0);
	HAL_Delay(2);
	HAL_GPIO_WritePin(PL_2_GPIO_Port , PL_2_Pin , 1);

	serial_data_temp1 |=  HAL_GPIO_ReadPin(Shift_data_out_GPIO_Port, Shift_data_out_Pin) ;

	for(int i = 1; i < 8 ; i++)
	{
		HAL_GPIO_WritePin(CLK_GPIO_Port , CLK_Pin , 1);
	    HAL_Delay(2);
        HAL_GPIO_WritePin(CLK_GPIO_Port , CLK_Pin , 0);
        HAL_Delay(2);
	    temp = HAL_GPIO_ReadPin(Shift_data_out_GPIO_Port, Shift_data_out_Pin);
	    serial_data_temp1 |=  HAL_GPIO_ReadPin(Shift_data_out_GPIO_Port, Shift_data_out_Pin) << (i);
	}

	HAL_GPIO_WritePin(EN_2_GPIO_Port , EN_2_Pin , 1);
	serial_data_temp = ~ serial_data_temp ;
	serial_data = serial_data_temp ;
}


void serial_data_read2()
{
	HAL_GPIO_WritePin(EN_1_GPIO_Port , EN_1_Pin , 0);
	HAL_GPIO_WritePin(EN_2_GPIO_Port , EN_2_Pin , 0);
	HAL_GPIO_WritePin(PL_1_GPIO_Port , PL_1_Pin , 0);
	HAL_GPIO_WritePin(PL_2_GPIO_Port , PL_2_Pin , 0);
	HAL_Delay(3);
	HAL_GPIO_WritePin(PL_1_GPIO_Port , PL_1_Pin , 1);
    HAL_GPIO_WritePin(PL_2_GPIO_Port , PL_2_Pin , 1);

    for(int i=0 ; i < 16 ; i++)
    {
    	temp = HAL_GPIO_ReadPin(Shift_data_out_GPIO_Port, Shift_data_out_Pin) ;
    	serial_data_temp |=  HAL_GPIO_ReadPin(Shift_data_out_GPIO_Port, Shift_data_out_Pin) << i;

        HAL_GPIO_WritePin(CLK_GPIO_Port , CLK_Pin , 1);
    	HAL_Delay(2);
        HAL_GPIO_WritePin(CLK_GPIO_Port , CLK_Pin , 0);
        HAL_Delay(2);


    }

	HAL_GPIO_WritePin(EN_2_GPIO_Port , EN_2_Pin , 1);
	HAL_GPIO_WritePin(EN_1_GPIO_Port , EN_1_Pin , 1);
	serial_data_temp = ~ serial_data_temp ;
	serial_data = serial_data_temp ;
}



void dac80501()
{

HAL_I2C_Master_Transmit(&hi2c1, DAC8051_add_w, &tx_data1, 1, 100);
HAL_I2C_Master_Receive(&hi2c1, DAC8051_add_r, &rx_data1, 2 ,100);

HAL_I2C_Master_Transmit(&hi2c1, DAC8051_add_w, &tx_data2, 1, 100);
HAL_I2C_Master_Receive(&hi2c1, DAC8051_add_r, &rx_data1, 2 ,100);

HAL_I2C_Master_Transmit(&hi2c1, DAC8051_add_w, &tx_data1, 1, 100);
HAL_I2C_Master_Receive(&hi2c1, DAC8051_add_r, &rx_data1, 2 ,100);

HAL_I2C_Master_Transmit(&hi2c1, DAC8051_add_w, &tx_data3, 1, 100);
HAL_I2C_Master_Receive(&hi2c1, DAC8051_add_r, &rx_data1, 2 ,100);

}




















uint8_t setValue(uint16_t value){
	return MCP4725_setValue(&myMCP4725, value, MCP4725_FAST_MODE, MCP4725_POWER_DOWN_OFF);
}


void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
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
  MX_I2C1_Init();
  MX_TIM1_Init();

  HAL_TIM_Base_Start(&htim1);
  /* USER CODE BEGIN 2 */
//  myMCP4725 = MCP4725_init(&hi2c1, MCP4725A0_ADDR_A00, 3.3);
//  		// Check the connection:
//  		if(MCP4725_isConnected(&myMCP4725)){
//  			is_mcp_connected = 1;
//  		}
//  		else{
//  			is_mcp_connected = 0;
////  			while(1);
//  		}

  		HAL_GPIO_WritePin(CLK_GPIO_Port , CLK_Pin , 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  parallel_data_read();
//	  serial_data_read();
//	  serial_data_read2();

	  dac80501() ;




//	  digitalWrite(EnablePin, HIGH);
//	      digitalWrite(LoadPin, LOW);
//	      delayMicroseconds(5);
//	      digitalWrite(LoadPin, HIGH);
//	      digitalWrite(EnablePin, LOW);
//
//	      for(int i = 0; i < DATA_WIDTH; i++)
//	      {
//	          bitVal = digitalRead(DataPin);
//	          bytesVal |= (bitVal << ((DATA_WIDTH-1) - i));
//
//	          digitalWrite(ClockPin, HIGH);
//	          delayMicroseconds(5);
//	          digitalWrite(ClockPin, LOW);
//	      }
//	  serial_data_temp = 0;
//
//	           	  	HAL_GPIO_WritePin(EN_1_GPIO_Port , EN_1_Pin , 1);
//	      	     	HAL_GPIO_WritePin(PL_1_GPIO_Port , PL_1_Pin , 0);
////	      	     	HAL_Delay(5);
//	      	     	delay_us(5);
//	      	     	HAL_GPIO_WritePin(PL_1_GPIO_Port , PL_1_Pin , 1);
//	      	     	HAL_GPIO_WritePin(EN_1_GPIO_Port , EN_1_Pin , 0);
//
//	      	     	HAL_Delay(1000);
//
//	      	     	for (int i=0 ; i<7 ; i++)
//	      	     	{
//	      	     	 HAL_Delay(5);
//	      	     		temp = HAL_GPIO_ReadPin(Shift_data_out_GPIO_Port, Shift_data_out_Pin);
//	      	     	    serial_data_temp |=  HAL_GPIO_ReadPin(Shift_data_out_GPIO_Port, Shift_data_out_Pin) << (i);
//	      	     	 HAL_Delay(3);
//
//
//	      	         HAL_GPIO_WritePin(CLK_GPIO_Port , CLK_Pin , 1);
////	      	        HAL_Delay(10);
//	      	        delay_us(5);
//	      	         HAL_GPIO_WritePin(CLK_GPIO_Port , CLK_Pin , 0);
////	      	        HAL_Delay(10);
//
//	      	     	}
//
//	serial_data = serial_data_temp ;





















//     	  	HAL_GPIO_WritePin(EN_1_GPIO_Port , EN_1_Pin , 1);
//	     	HAL_GPIO_WritePin(PL_2_GPIO_Port , PL_2_Pin , 0);


//	        HAL_GPIO_WritePin(CLK_GPIO_Port , CLK_Pin , 0);
//	  		HAL_Delay(2);
//	  		HAL_GPIO_WritePin(CLK_GPIO_Port , CLK_Pin , 1);
//	  		HAL_Delay(2);



//	  setValue(parallel_data >> 4);




//	  	input0 = HAL_GPIO_ReadPin(Input_0_GPIO_Port, Input_0_Pin) ;
//	  	input0 = HAL_GPIO_ReadPin(Input_1_GPIO_Port, Input_1_Pin) ;
//	  	input0 = HAL_GPIO_ReadPin(Input_2_GPIO_Port, Input_2_Pin)  ;
//	  	input0 = HAL_GPIO_ReadPin(Input_3_GPIO_Port, Input_3_Pin) ;
//	  	input0 = HAL_GPIO_ReadPin(Input_4_GPIO_Port, Input_4_Pin)  ;
//	  	input0 = HAL_GPIO_ReadPin(Input_5_GPIO_Port, Input_5_Pin)  ;
//	  	input0 = HAL_GPIO_ReadPin(Input_6_GPIO_Port, Input_6_Pin) ;
//	  	input0 = HAL_GPIO_ReadPin(Input_7_GPIO_Port, Input_7_Pin)  ;
//	  	input0 = HAL_GPIO_ReadPin(Input_8_GPIO_Port, Input_8_Pin) ;
//	  	input0 = HAL_GPIO_ReadPin(Input_9_GPIO_Port, Input_9_Pin)  ;
//		input0 = HAL_GPIO_ReadPin(Input_10_GPIO_Port, Input_10_Pin) ;
//		input0  = HAL_GPIO_ReadPin(Input_11_GPIO_Port, Input_11_Pin) ;
//		input0  = HAL_GPIO_ReadPin(Input_12_GPIO_Port, Input_12_Pin)  ;
//		input0  = HAL_GPIO_ReadPin(Input_13_GPIO_Port, Input_13_Pin)  ;
//		input0 = HAL_GPIO_ReadPin(Input_14_GPIO_Port, Input_14_Pin)  ;
//		input0 = HAL_GPIO_ReadPin(Input_15_GPIO_Port, Input_15_Pin) ;

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EN_1_Pin|EN_2_Pin|PL_1_Pin|PL_2_Pin
                          |CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Input_10_Pin */
  GPIO_InitStruct.Pin = Input_10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Input_10_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Input_0_Pin Input_1_Pin Input_2_Pin Input_3_Pin
                           Input_4_Pin Input_5_Pin Input_6_Pin Input_7_Pin */
  GPIO_InitStruct.Pin = Input_0_Pin|Input_1_Pin|Input_2_Pin|Input_3_Pin
                          |Input_4_Pin|Input_5_Pin|Input_6_Pin|Input_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Input_8_Pin Input_9_Pin Input_11_Pin Input_12_Pin
                           Input_13_Pin Input_14_Pin Input_15_Pin */
  GPIO_InitStruct.Pin = Input_8_Pin|Input_9_Pin|Input_11_Pin|Input_12_Pin
                          |Input_13_Pin|Input_14_Pin|Input_15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_1_Pin EN_2_Pin PL_1_Pin PL_2_Pin
                           CLK_Pin */
  GPIO_InitStruct.Pin = EN_1_Pin|EN_2_Pin|PL_1_Pin|PL_2_Pin
                          |CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Shift_data_out_Pin */
  GPIO_InitStruct.Pin = Shift_data_out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Shift_data_out_GPIO_Port, &GPIO_InitStruct);

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
