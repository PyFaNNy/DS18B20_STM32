/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "string.h"
#include "stdio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void USART3_UART_Init_115200(void);
static void USART3_UART_Init_9600(void);
int32_t USART2_ReceiveChar(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	//#define SIMULATOR
	uint8_t tick1000mc;
	const uint16_t MAX_LEN = 10;
	
	const uint8_t Match_Rom = 0x55;			// Select sensor
	const uint8_t Skip_Rom = 0xCC;				// ALL sensors
	const uint8_t Search_ROM = 0xF0; 		// Search sensors
	const uint8_t ROM_Read_Rom = 0x33;		// Read serial number
	const uint8_t Start_Mesuare = 0x44;	// Start measure temp
	const uint8_t Conver_Temp = 0x48;		// Start measure temp
	const uint8_t ReadCommand = 0xFF;		// Read
	const uint8_t ReadTemp = 0xBE;			// Read temp
	uint8_t counter =0;
	char inputStr[MAX_LEN+1];
	char ready = '0';
	
	
	//DS18B20
	void one_wire_tx_rx(uint8_t *rx_s, const uint8_t *tx_s, int size) 
	{
		for(int i=0; i<size; i++)
		{
			//send byte data time slot
			while(!(USART3->SR & UART_FLAG_TXE));
			USART3->DR=tx_s[i]&0xFF;
			
			//wait finish transmit
			while(!(USART3->SR & UART_FLAG_TC));
			
			//wait receiver
			while(!(USART3->SR & UART_FLAG_RXNE));
			
			rx_s[i]= USART3_ReceiveChar();
		}
	}
	
	uint8_t reset_1_wire()
	{
		USART3_UART_Init_9600();
		uint8_t result;
		one_wire_tx_rx(&result, &Search_ROM, 1);
		USART3_UART_Init_115200();
		return result;
	}
	
	uint64_t getId(void)
	{
		//reset
		uint8_t wire = reset_1_wire();
		if(wire == Search_ROM)
		{
			return 0;
		}
		//send command
		uint8_t result[8];
		uint8_t commandReadRom[8] = {0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00};
		one_wire_tx_rx(result, commandReadRom, 8);
		
		//get id
		uint64_t id = 0;
		for(int i = 0; i < 64; i++) {
			uint8_t readByte;
			one_wire_tx_rx(&readByte, &ReadCommand, 1);
			uint8_t x = 0;
			if(readByte == ReadCommand)
			{
				x =1;
			}
			id >>= 1;
			if (x) id |= 0x8000000000000000;
		} 
		return id;
	}
	
	float getT(void)
	{
		float T = -100;
		uint8_t wire = reset_1_wire();
		if(wire == Search_ROM)
		{
			return -100;
		}
		
		uint8_t result[8];
		uint8_t commandSkipRom[8] = {0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF};
		one_wire_tx_rx(result, commandSkipRom, 8);
		
		uint8_t commandReadTemp[8] = {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF};
		one_wire_tx_rx(result, commandReadTemp, 8);
		
		uint16_t cod_temperature = 0;
		for(int i=0; i<16; i++)
		{
			uint8_t result;
			one_wire_tx_rx(&result, &ReadCommand, 1);
			cod_temperature >>= 1;
			if (result == 0xff) cod_temperature |= 0x8000;
		}

		T = cod_temperature/16.0;
		return T;
	}
	
	int8_t startT()
	{
		uint8_t wire = reset_1_wire();
		if(wire == Search_ROM)
		{
			return -100;
		}
		
		uint8_t result[8];
		uint8_t commandSkipRom[8] = {0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF};
		one_wire_tx_rx(result, commandSkipRom, 8);
		
		uint8_t commandStartMesuare[8] = {0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0x00};
		one_wire_tx_rx(result, commandStartMesuare, 8);
		
		return 0;
	}
	
	
	
	uint8_t crc(uint8_t *buff)
	{
		uint8_t crc = 0;
		for (uint8_t i=0; i <8; ++i)
		{
			uint8_t data = buff[i];
			for (uint8_t j =0; j <8; ++j)
			{
				uint8_t tmp = (crc ^ data) & 0x01;
				if(0 != tmp) crc ^= 0x18;
				crc = (crc>>1) & 0x7F;
				if (0!= tmp) crc |=0x80;
				data = data >> 1;
			}
		}
		return crc;
	}
	//Finish DS18B20
		
	uint32_t ITM_puts(const char *s)
	{
		size_t i=0;
		while (s[i]) ITM_SendChar (s[i++]);
		return i;
	}
	
	
	
	//USART2
	uint32_t USART2_SendChar(uint32_t ch)
	{
			while(!(USART2->SR & UART_FLAG_TXE));
			USART2->DR=ch&0xFF;
			return ch;
	}
	
	void USART2_Send_String(char* str)
	{
    int i = 0;
    while(str[i])
        USART2_SendChar(str[i++]);
	}
	
	int32_t USART2_ReceiveChar(void)
	{
		if((USART2->SR & UART_FLAG_RXNE))
		{
			return USART2->DR;
		}
		else
		{
			return 0x100;
		}
	}
	//Finish USART2
	
	//USART3
	uint32_t USART3_SendChar(uint32_t ch)
	{
			while(!(USART3->SR & UART_FLAG_TXE));
			USART3->DR=ch&0xFF;
			return ch;
	}
	
	void USART3_Send_String(char* str)
	{
    int i = 0;
    while(str[i])
        USART3_SendChar(str[i++]);
	}
	
	int32_t USART3_ReceiveChar()
	{
		if((USART3->SR & UART_FLAG_RXNE))
		{
			return USART3->DR;
		}
		else
		{
			return 0x100;
		}
	} 
	//Finish USART3
	
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
  MX_USART2_UART_Init();
  USART3_UART_Init_115200();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint16_t counterTemperature =0;
	uint32_t err =0;
	float result=0;
	char str[MAX_LEN+1];
  while (1)
  {
			if(tick1000mc)
			{
				LED_RED_GPIO_Port->ODR ^= LED_RED_Pin;
				counterTemperature++;
				//USART2_SendChar(counterTemperature);
				
				uint64_t id = getId();
				uint8_t sum = crc((uint8_t *)&id);
				if(sum)
				{
					//USART2_Send_String("\nNo impulse ID\n\r");
					ITM_puts("\nNo impulse ID\n\r");
					tick1000mc = 0;
					continue;
				}
				else
				{
					char ss[50];
					sprintf(ss, "Id - 0x%llx Sum - %d ", id, sum);
					//USART2_Send_String(ss);
					ITM_puts(ss);
				}
								
				float temperature = getT();
				if(temperature == -100)
				{
					//USART2_Send_String("\nNo impulse T\n\r");
					ITM_puts("\nNo impulse T\n\r");
					tick1000mc = 0;
					continue;
				}
				else
				{
					char ss[50];
					sprintf(ss, "Temperature - %f\n\r", temperature);
					//USART2_Send_String(ss);
					ITM_puts(ss);
				}
					
				int8_t startt = startT();
				if(startt == -100)
				{
					//USART2_Send_String("\nNo impulse StartT\n\r");
					ITM_puts("\nNo impulse StartT\n\r");
					tick1000mc = 0;
					continue;
				}
				
				tick1000mc = 0;
			}
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void USART3_UART_Init_9600(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

static void USART3_UART_Init_115200(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_GREEN_Pin|LED_ORANGE_Pin|LED_RED_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : KEY_BLUE_Pin */
  GPIO_InitStruct.Pin = KEY_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY_BLUE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_ORANGE_Pin LED_RED_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_ORANGE_Pin|LED_RED_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
	#ifndef SIMULATOR
  __disable_irq();
  while (1)
  {
  }
	#endif
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
