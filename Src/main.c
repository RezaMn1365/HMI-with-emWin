/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "GUI.h"
#include "HX8352B-F4-PAR.h"
#include "WindowDLG.h"
#include "GUITDRV_ADS7846.h"
#include "XPT2046-REZ.h"
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
CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
uint8_t		TDATA[3], BUSY, TCMD[6];
uint16_t	Result, TCSDATA[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_CRC_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	GUI_Init();		
	//GUI_SetFont(&GUI_Font32B_1);
	//GUI_SetBkColor(GUI_BLUE);
	//GUI_SetColor(GUI_BLACK);
	//GUI_SetLineStyle(GUI_LS_DASHDOTDOT);
	//GUI_CURSOR_Select(&GUI_CursorArrowM);
	//GUI_CURSOR_SetPosition(10, 150);
	//GUI_Clear();	
	//GUI_SetLineStyle(GUI_LS_SOLID);		
	//LCD_RECT GUI_RECT = { 0 };
	//GUI_RECT.x0 = 70;
	//GUI_RECT.x1 = 350;
	//GUI_RECT.y0 = 70;
	//GUI_RECT.y1 = 160;
	
	//GUI_DrawRect(60, 60, 350, 160);
	
	//GUI_DispStringInRect("Hello.........World!", &GUI_RECT, GUI_TA_HCENTER);
	
	//GUI_CURSOR_SetPosition(20, 50);
	//GUI_CURSOR_Show();	
	
	//
	MainTask();  // CreateWindow();
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
	
		
		//CreateWindow();
		HAL_Delay(100);	
		
		
	
		/*
		 
		
		SendCmd(0XD0);
		TCSDATA[3] = GetResult();	
		SetCS(1);
		
		
		
		HAL_Delay(100);
		
			TOUCH_X_Init				// S / A2 A1 A0 / MODE / SER_DFR / PD1 PD0
		TCMD[0] = 0X90;   //1 / 101        / 1   / 0        / 00       X-POSITION
		TCMD[1] = 0X00;   //1 / 001        / 1   / 0        / 00       Y-POSITION    
		TCMD[2] = 0X00;
		
		TCMD[3] = 0XD0;
		TCMD[4] = 0X00;
		TCMD[5] = 0X00;
		
		//YY :if (HAL_GPIO_ReadPin(GPIOA, PENIRQ_Pin)) {goto YY; }	else
		
		HAL_GPIO_WritePin(GPIOA, TCS_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);	
		
		HAL_SPI_TransmitReceive(&hspi1, &TCMD[0], &TDATA[0], 1, 15);
		HAL_SPI_TransmitReceive(&hspi1, &TCMD[1], &TDATA[1], 2, 15);
		TCSDATA[3] = ((TDATA[1] << 4) | (TDATA[2] >> 4)) & 0X0FFF;
		
		HAL_SPI_TransmitReceive(&hspi1, &TCMD[3], &TDATA[0], 1, 15);
		HAL_SPI_TransmitReceive(&hspi1, &TCMD[4], &TDATA[1], 2, 15);
		TCSDATA[2] = ((TDATA[1] << 4) | (TDATA[2] >> 4)) & 0X0FFF;
	
		HAL_GPIO_WritePin(GPIOA, TCS_Pin, GPIO_PIN_SET);
		
		
		
		
							// S / A2 A1 A0 / MODE / SER_DFR / PD1 PD0
		TCMD[0] = 0XD0;   //1 / 101        / 1   / 0        / 00       X-POSITION
		TCMD[1] = 0X90;   //1 / 001        / 1   / 0        / 00       Y-POSITION    
		TCMD[2] = 0X00;
		 
		HAL_SPI_TransmitReceive(&hspi1, &TCMD[0], &TDATA[0], 1, 5);
		HAL_SPI_TransmitReceive(&hspi1, &TCMD[2], &TDATA[1], 1, 5);
		TCSDATA[0] = TDATA[0];
		TCSDATA[1] = TDATA[1];
		TCSDATA[3] = ((TCSDATA[0] << 4) | (TCSDATA[1] >> 4)) & 0X0FFF;
		 
		 
		 	XX:
		if (HAL_GPIO_ReadPin(GPIOA, BUSY_Pin) == 0)
		{
			HAL_SPI_Receive(&hspi1, &TDATA[0], 1, 25);
			HAL_SPI_Receive(&hspi1, &TDATA[1], 1, 25);
			TCSDATA = (TDATA[0] << 4 | TDATA[1]>>4) & 0X0FFF;
		}
		else goto XX;
		
		
		
		HAL_SPI_Transmit(&hspi1, &TCMD[1], 1, 5);				
		YY :
		if (HAL_GPIO_ReadPin(GPIOA, BUSY_Pin) == 0)
		{}		else goto YY;
		
			HAL_Delay(40);
		GUITDRV_ADS7846_Exec();
		HAL_Delay(40);		
	
		 HAL_Delay(10);
		GUI_DispStringAt("Hello.........World!", 50, 10);
		HAL_Delay(10);
		GUI_DispStringAt("Hello.........World!", 50, 50);
		HAL_Delay(10);
		GUI_DispStringAt("Hello.........World!", 50, 100);
		HAL_Delay(10);
		GUI_DispStringAt("Hello.........World!!", 50, 150);
		HAL_Delay(10);
		GUI_DispStringAt("Hello.........World!!", 50, 200);
		HAL_Delay(10);
		GUI_Clear();
		HAL_Delay(10);
		//GUI_DrawRect(50, 50, 350, 160);
		//GUI_DispStringInRect("Hello.........World!", &GUI_RECT, GUI_TA_HCENTER);
		HAL_Delay(1000);
		GUI_Clear();
		 
	  
		 E1 = ((address << 3) & 0XFF80);
		 D1 = (address << 14) & 0XC000;
		 D2 = (address >> 2) & 0X0003;
		 D3 = (address >> 5) & 0X0700;
		 D4 = (D1 | D2 | D3) | 0X38FC;
		 GPIOE->ODR = E1;
		 GPIOD->ODR = D4;
		 HAL_Delay(20);
		 GPIOD->ODR = D4 & CS_MASK; 		//CS=LOW RS=HIGH WR=HIGH RD=HIGH
		 HAL_Delay(30);
		 GPIOD->ODR = D4 & (RS_MASK & CS_MASK);  		//CS=LOW RS=LOW WR=HIGH RD=HIGH
		 HAL_Delay(20);
		 GPIOD->ODR = D4 & (WR_MASK & RS_MASK & CS_MASK); 		//CS=LOW RS=LOW WR=LOW RD=HIGH
		 HAL_Delay(20);
		 GPIOD->ODR = (D4 & (RS_MASK & CS_MASK)) | (~WR_MASK);   	//CS=LOW RS=LOW WR=HIGH RD=HIGH		
		 HAL_Delay(20);
		 GPIOD->ODR = (D4 & (CS_MASK)) | (~RS_MASK); 	//CS=LOW RS=HIGH WR=HIGH RD=HIGH
		 HAL_Delay(20);
		 */
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
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
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
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
	
  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TCS_GPIO_Port, TCS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |LCD_RST_Pin|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0
                          |GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pins : PENIRQ_Pin BUSY_Pin */
  GPIO_InitStruct.Pin = PENIRQ_Pin|BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TCS_Pin */
  GPIO_InitStruct.Pin = TCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(TCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PE7 PE8 PE9 PE10
                           PE11 PE12 PE13 PE14
                           PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11
                           LCD_RST_Pin PD14 PD15 PD0
                           PD1 PD4 PD5 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |LCD_RST_Pin|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0
                          |GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
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
