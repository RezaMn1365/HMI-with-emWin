/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 SRM.
   *
  ******************************************************************************
  */
/* USER CODE END Header */
// #include "main.h"
// #include "GUITDRV_ADS7846.h"

// /* Includes ------------------------------------------------------------------*/

// uint16_t	Result1;
// extern SPI_HandleTypeDef hspi1;
// GUITDRV_ADS7846_LAST_VAL TCS_LAST_VALUE = { 0 } ;

// void SendCmd(uint8_t data)		// Sends a 8-bit command to the peripheral
// {	
// 	uint8_t TDATA[3];	
// 	TDATA[0] = data;
// 	TDATA[1] = 0;
// 	TDATA[2] = 0;
// 	uint8_t RDATA[3];
// 	HAL_SPI_TransmitReceive(&hspi1, &TDATA[0], &RDATA[0], 3, 105);
// 	Result1 = ((RDATA[1] << 4) | (RDATA[2] >> 4)) & 0X0FFF;	
// 	/*
// 	TDATA[0] = (data & 0xF0);
// 	switch (TDATA[0])
// 	{
// 	case 0XD0 :
// 		TCS_LAST_VALUE.xPhys = Result1; 
// 		break;
// 	case 0x90 : 
// 		TCS_LAST_VALUE.yPhys = Result1; 
// 		break;
// 	case 0xB0 : 
// 		TCS_LAST_VALUE.z1Phys = Result1;
// 		break;
// 	case 0xC0 : 
// 		TCS_LAST_VALUE.z2Phys = Result1;
// 		break;
// 	default:
// 		break;
// 	}*/
	
// }
// 	void SetCS(char mode)		// Set chip select line. OnOff == 1 means peripheral selected
// 	{
// 		if (mode == 1)
// 		{
// 			HAL_GPIO_WritePin(GPIOA, TCS_Pin, GPIO_PIN_RESET);
// 		}	
// 	}

// 	char GetBusy(void)			// Retrieves the status of the busy line. 0: Not busy; 1: Busy
// 	{	
// 		if ((GPIOA->IDR & 0X0010) == 0){ return (0); }
// 		else {return (1);}	
// 	}

// 	char GetPENIRQ(void) {	         // Retrieves the status of the PENIRQ line to detect a touch event.
// 		if((GPIOA->IDR & 0X0004) == 0)
// 		{	return (1);}
// 			//TCS_LAST_VALUE.PENIRQ = 0; }
// 		else
// 			return (0);
// 			//TCS_LAST_VALUE.PENIRQ = 1; }	
// 	}

// uint16_t GetResult(void)		// Retrieves the result of the AD conversion. 4 dummy bytes have to be shifted out to the left.
// {	
// 	//GUITDRV_ADS7846_GetLastVal(&TCS_LAST_VALUE);
// 	return (Result1);
// }




// extern SPI_HandleTypeDef hspi1;


#include <stm32f427xx.h>
#include <stm32f4xx_hal_def.h>
#include "main.h"
#include "GUITDRV_ADS7846.h"
#include <GUI.h>

extern SPI_HandleTypeDef hspi1;


extern uint16_t	Result;
extern uint16_t	TCSDATA[4];
uint8_t ii = 0, iv;
uint16_t TouchAvgData[48];
 void SendCmd(uint8_t data)
{
	/* Sends a 8-bit command to the peripheral */
	HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, &data, 1, 100);
	if (status != HAL_OK) {
		//DEBUG_BREAK();
	}
}
 U16 GetResult(void)
{
	/* Retrieves the result of the AD conversion. */
	uint8_t buffer[2] = { 0 };
	uint8_t dummy[2] = { 0 };
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, dummy, buffer, 2, 100);
	if (status != HAL_OK) {
		//DEBUG_BREAK();
	}
	TCSDATA[0] = buffer[0];
	TCSDATA[1] = buffer[1];
	
	uint16_t result = ((buffer[0] << 4) | (buffer[1]) >> 4) & 0X0F80; // be_to_h_u16(buffer);
	uint8_t max = 16;
	
	if (ii < max)
	{
		
		TouchAvgData[ii] = result;
		ii++;
		
	}else
	{
		Result = 0;
		ii = 0;
		for (iv = 0; iv < max; iv++)
		{
			Result += TouchAvgData[iv];
		}
		Result = Result / max;
	}
	 
	//result >>= 3;
	Result = result;
	return Result;
}
 char GetBusy(void)
{
	/* Retrieves the status of the busy line. 0: Not busy; 1: Busy */
	GPIO_PinState gpio_state = HAL_GPIO_ReadPin(GPIOA, BUSY_Pin);
	return (gpio_state == GPIO_PIN_SET);
	//return 0;
}
 void SetCS(char OnOff)
{
	/* Set chip select line. OnOff == 1 means peripheral selected */
	HAL_GPIO_WritePin(GPIOA, TCS_Pin, (OnOff == 1) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
 char GetPENIRQ(void)
{
	/* Retrieves the status of the PENIRQ line to detect a touch event.
 	* return 1 if a touch event is recognized and 0 if not. */
	GPIO_PinState gpio_state = HAL_GPIO_ReadPin(GPIOA, PENIRQ_Pin);
	return (gpio_state == GPIO_PIN_RESET);
}
// static GUITDRV_ADS7846_CONFIG touch_config = {
// 	.pfSendCmd = platform_touch_send_cmd,
// 	.pfGetResult = platform_touch_get_result,
// 	.pfGetBusy = platform_touch_get_busy,
// 	.pfSetCS = platform_touch_set_cs,
// 	.Orientation = GUI_SWAP_XY,
// 	.xLog0 = 0,
// 	.xLog1 = (400 - 1),
// 	.xPhys0 = 0,
// 	.xPhys1 = 4095,
// 	.yLog0 = 0,
// 	.yLog1 = (240 - 1),
// 	.yPhys1 = 0,
// 	.yPhys0 = 4095,
// 	.pfGetPENIRQ = platform_touch_get_penirq,
// };
