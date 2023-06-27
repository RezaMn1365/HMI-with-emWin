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
#include "HX8352B-F4-PAR.h"
#include "GUI.h"

						        // GPIOx bit 7  6  5    4     3  2  1  0
								//			 x  x  x  RSOUT   WR  RD  CS  RS
								
								
#define CS_MASK 0XFF7F
#define WR_MASK 0XFFDF
#define RD_MASK 0XFFEF
#define RS_MASK 0XF7FF
#define RST_MASK 0XEFFF

#define PORTxDATA_MASK 0X1FF0


void LCD_DATA_GPIO_IN()
{
 GPIO_InitTypeDef GPIO_InitStruct = {0};
 
  
  /*Configure GPIO pins : PE7 PE8 PE9 PE10 
                           PE11 PE12 PE13 PE14 
                           PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;  
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11 
                           LCD_RST_Pin PD14 PD15 PD0 
                           PD1 PD4 PD5 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void LCD_DATA_GPIO_OUT()
{
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	HAL_GPIO_WritePin(GPIOE,
		GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10 
	                      |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
	                      |GPIO_PIN_15,
		GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD,
		GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
	                        |LCD_RST_Pin|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0 
	                        |GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7,
		GPIO_PIN_SET);

	/*Configure GPIO pins : PE7 PE8 PE9 PE10 
	                         PE11 PE12 PE13 PE14 
	                         PE15 */
	GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 
	                        | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 
	                        | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : PD8 PD9 PD10 PD11 
	                         LCD_RST_Pin PD14 PD15 PD0 
	                         PD1 PD4 PD5 PD7 */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 
	                        | LCD_RST_Pin | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_0 
	                        | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}



void us_Delay(uint16_t US)
{
	while (US>0)
	{
		US--;
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();		
	}
}


void LCDRST(void)
{
	GPIOD->ODR = RST_MASK;
	us_Delay(200);
	GPIOD->ODR = 0xFFFF;
	us_Delay(20);
}


void Lcd_Write_Reg(uint16_t address) {	
	uint16_t E1,D1,D2,D3,D4;
	E1 = ((address<<3) & 0XFF80);
	//D1 = (address << 14) & 0XC000;
	//D2 = (address >> 2) & 0X0003;
	//D3 = (address >> 5) & 0X0700;
	//D4 = (D1 | D2 | D3) | (0X38FC);
	D4 = ((address << 14) & 0XC000) | ((address >> 2) & 0X0003) |((address >> 5) & 0X0700) | (0X38FC);	 
	GPIOE->ODR = E1;
	GPIOD->ODR = D4;
	us_Delay(2);
	GPIOD->ODR = D4 & CS_MASK;		//CS=LOW RS=HIGH WR=HIGH RD=HIGH
	us_Delay(3);
	GPIOD->ODR = D4 & (RS_MASK & CS_MASK);  		//CS=LOW RS=LOW WR=HIGH RD=HIGH
	us_Delay(2);
	GPIOD->ODR = D4 & (WR_MASK & RS_MASK & CS_MASK); 		//CS=LOW RS=LOW WR=LOW RD=HIGH
	us_Delay(2);
	GPIOD->ODR = (D4 & (RS_MASK & CS_MASK)) | (~WR_MASK);  	//CS=LOW RS=LOW WR=HIGH RD=HIGH		
	us_Delay(2);
	GPIOD->ODR = (D4 & (CS_MASK)) | (~RS_MASK); 	//CS=LOW RS=HIGH WR=HIGH RD=HIGH
	//us_Delay(2);
}

void Lcd_Write_Data(uint16_t data) {
	uint16_t E1, D1, D2, D3, D4;
	E1 = ((data << 3) & 0XFF80);
	//D1 = (data << 14) & 0XC000;
	//D2 = (data >> 2) & 0X0003;
	//D3 = (data >> 5) & 0X0700;	
	//D4 = (D1 | D2 | D3) | 0X38FC;
	D4 = ((data << 14) & 0XC000) | ((data >> 2) & 0X0003) | ((data >> 5) & 0X0700) | (0X38FC);
	GPIOE->ODR = E1;
	GPIOD->ODR = D4;
	us_Delay(2);
	GPIOD->ODR = D4 & CS_MASK;		//CS=LOW RS=HIGH WR=HIGH RD=HIGH
	us_Delay(2);
	GPIOD->ODR = D4 & (WR_MASK & CS_MASK); 		//CS=LOW RS=HIGH WR=LOW RD=HIGH
	us_Delay(2);
	GPIOD->ODR = (D4 & CS_MASK) | (~WR_MASK); 		//CS=LOW RS=HIGH WR=HIGH RD=HIGH	
	//us_Delay(2);
}

uint16_t Lcd_Read_Data() {
	uint16_t E1, reg, D2,D3,D4,D5, D1=0X18B0;
	LCD_DATA_GPIO_IN();	
	//us_Delay(2);
	GPIOD->ODR = D1 & CS_MASK;					//CS=LOW RS=HIGH WR=HIGH RD=HIGH
	us_Delay(2);		
	GPIOD->ODR = D1 & RD_MASK & CS_MASK; 		//CS=LOW RS=HIGH WR=HIGH RD=LOW
	us_Delay(2);
	
	//E1 = ((GPIOE->IDR & 0XFF80) >> 3) & PORTxDATA_MASK;
	D2 = GPIOD->IDR & 0XC703;
	//D3 = ((D2 >> 14) & 0X0003);
	//D4 = ((D2 << 2) & 0X000C);
	//D5 = ((D2 << 5) & 0XE000);		
	reg = ((((GPIOE->IDR & 0XFF80) >> 3) & PORTxDATA_MASK) | ((D2 >> 14) & 0X0003) | ((D2 << 2) & 0X000C) | ((D2 << 5) & 0XE000));
	
	GPIOD->ODR = D1 ; 				//CS=LOW RS=HIGH WR=HIGH RD=HIGH		
	LCD_DATA_GPIO_OUT();	
	return reg;
}


void Lcd_Init()
{
	LCDRST();	  
	us_Delay(20000); 
	
	 
	/**
	 **
	 **GPIOD->ODR = D1 ;  							//CS=HIGH RS=HIGH WR=HIGH RD=HIGH
	us_Delay(5);
	GPIOD->ODR = D1 & CS_MASK; 					//CS=LOW RS=HIGH WR=HIGH RD=HIGH
	us_Delay(5);		
	GPIOD->ODR = D1 & RD_MASK & CS_MASK;  		//CS=LOW RS=HIGH WR=HIGH RD=LOW
	us_Delay(5);
	
	 
	GPIOD->ODR = D1 & RD_MASK & CS_MASK; 		//CS=LOW RS=HIGH WR=HIGH RD=LOW
	us_Delay(2);	
	E1 = GPIOE->IDR; 	//0XAB00;
	D2 = GPIOD->IDR;	//0XC001;
	GPIOD->ODR = (D1 | ~RD_MASK) & CS_MASK; 		//CS=LOW RS=HIGH WR=HIGH RD=HIGH
	us_Delay(2);
	
	
	  	// power settings
	 Lcd_Write_Reg(0x001A); //power control 1
	 // Switch the output factor of step-up circuit 2 for VGH and VGL voltage generation
	 Lcd_Write_Data(0x0004); //BT=0001  VLCD=5.3V  VCL=-VCI  3VLCD->VGH  -2VLCD->VGL  
	 
	 Lcd_Write_Reg(0x001B); //power control 2
	 // Specify the VREG1 voltage adjusting. VREG1 voltage is for gamma voltage setting.
	 //0=3.3 step 0.05 y=x*0.05+3.3 x=y-3.3/0.05
	 Lcd_Write_Data(0x0018); //VRH=4.5V    will be stable if <= VLCD-0.3V and VREF <= VLCD-0.3V

	 Lcd_Write_Reg(0x0023); //VCOM1
	 // Set the VCOM offset voltage.
	 Lcd_Write_Data(0x0080); //VMF=original VML/VMH
	 Lcd_Write_Reg(0x0025); //VCOM3
	 // Set the VCOML voltage (Low level voltage of VCOM)
	 //0=-2.5 step 0.015 y=x*0.015-2.5 x=y+2.5/0.0.15
	 Lcd_Write_Data(0x002A); //VML=VCOML=-1.87V  will be stable if VREF <= VLCD -0.3V
	 Lcd_Write_Reg(0x0024); //VCOM2
	 // Set the VCOMH voltage (High level voltage of VCOM)
	 //0=2.5 step 0.015 y=x*0.015+2.5 x=y-2.5/0.015
	 Lcd_Write_Data(0x002A); //VMH=VCOMH=3.13V   will be stable if VREF <= VLCD -0.3V

	 // power on
	 Lcd_Write_Reg(0x0019); //OSC control 2
	 Lcd_Write_Data(0x0001); //OSC_EN=1
	 us_Delay(20);

	 Lcd_Write_Reg(0x001F); //power control 6
	 // GAS_EN(1)  VCO_MG  x  PON  DK(1)  XDK(1)  DDV_DH_T_RI  STB(1)
	 // VCOMG: When VCOMG = ‘1’, VCOML voltage can output to negative voltage (1.0V ~VCL+0.5V).
	 // When VCOMG = ‘0’, VCOML outputs GND and VML[7:0] setting are invalid. Then, low power consumption is accomplished.
	 // PON: Specify on/off control of step-up circuit 2 for VCL, VGL voltage generation
	 // DK: Specify on/off control of step-up circuit 1 for VLCD voltage generation.
	 // XDK, DDVDH_TRI: Specify the ratio of step-up circuit for VLCD voltage generation.
	 // ^^^^^^^^^^^^^^ uses ext capacitors? need experiment may be
	 Lcd_Write_Data(0x008C); //STB=0

	 Lcd_Write_Reg(0x001C); // power control 3
	 // Adjust the amount of current driving for the operational amplifier in the power supply circuit.
	 // When the amount of fixed current is increased, the LCD driving capacity
	 // and the display quality are high, but the current consumption is increased.
	 Lcd_Write_Data(0x0004); // medium
	 us_Delay(20); //5

	 Lcd_Write_Reg(0x001F); //power control 6
	 Lcd_Write_Data(0x0084); //DK=0
	 us_Delay(20); //5

	 Lcd_Write_Reg(0x001F); //power control 6
	 Lcd_Write_Data(0x0094); //PON=1
	 us_Delay(20); //5

	 Lcd_Write_Reg(0x001F); //power control 6
	 Lcd_Write_Data(0x00D4); //VCOMG=1
	 us_Delay(20); //5

	 // display on
	 Lcd_Write_Reg(0x0028); //display control 3
	 Lcd_Write_Data(0x0038); //GON+DTE=11=Gate Output VGH/VGL, D10=10=Source Output - =PT(0,0)
	 us_Delay(40); //40

	 Lcd_Write_Reg(0x0028); //display control 3
	 Lcd_Write_Data(0x003C); //D10=11=Source Output - Display
	 us_Delay(40); //40

	 // misc settings
	 Lcd_Write_Reg(0x0016); //memory access
	 // MY MX MV x BGR SM SS GS
	 // MY mirror Y
	 // MX mirror X
	 // MV = like rotates memory by 90 degree, so that part is unused
	 Lcd_Write_Data(0b00001000);
	us_Delay(40);  //40 
	  

	 Lcd_Write_Reg(0x0017); //COLMOD
	 Lcd_Write_Data(0x0055); // 16bpp
	
	us_Delay(40);  //40
	
	//Gamma 2.2 Setting
    Lcd_Write_Reg(0x0040); Lcd_Write_Data(0x0000);	
	Lcd_Write_Reg(0x0041); Lcd_Write_Data(0x0009);
	Lcd_Write_Reg(0x0042); Lcd_Write_Data(0x0012);
	Lcd_Write_Reg(0x0043); Lcd_Write_Data(0x0004);
	Lcd_Write_Reg(0x0044); Lcd_Write_Data(0x0000);
	Lcd_Write_Reg(0x0045); Lcd_Write_Data(0x0023);       //
	Lcd_Write_Reg(0x0046); Lcd_Write_Data(0x0003);
	Lcd_Write_Reg(0x0047); Lcd_Write_Data(0x005E);       //
	Lcd_Write_Reg(0x0048); Lcd_Write_Data(0x0000);
	Lcd_Write_Reg(0x0049); Lcd_Write_Data(0x0000);
	Lcd_Write_Reg(0x004A); Lcd_Write_Data(0x0000);
	Lcd_Write_Reg(0x004B); Lcd_Write_Data(0x0000);
	Lcd_Write_Reg(0x004C); Lcd_Write_Data(0x0000);
	Lcd_Write_Reg(0x004D); Lcd_Write_Data(0x0000);
	Lcd_Write_Reg(0x004E); Lcd_Write_Data(0x0000);
	Lcd_Write_Reg(0x0057); Lcd_Write_Data(0x004F);       //

	  
  
	  */
	  
	Lcd_Write_Reg(0x0000); Lcd_Write_Data(0x0001);
	us_Delay(2);
    
	Lcd_Write_Reg(0x001A); Lcd_Write_Data(0x0004);       // BT[3:0]=0100, VCL=-VCI; VGH=VCI+2DDVDH; VGL=-2DDVDH
	Lcd_Write_Reg(0x001B); Lcd_Write_Data(0x0001);       // 3.3VOLT              VRH[4:0]=0Ch, VREG1=(2.5v*1.9)=4.75V		VCIRE=1; 
	// VCOM offset
	  
	  	  
	Lcd_Write_Reg(0x0023); Lcd_Write_Data(0x0000);       // SELVCM=0, R24h and R25h selects VCOM
	Lcd_Write_Reg(0x0024); Lcd_Write_Data(0x0040);      // VCM[6:0]=1000000, VCOMH voltage=VREG1*0.748 (originally 5F)
	Lcd_Write_Reg(0x0025); Lcd_Write_Data(0x000F);      // VDV[4:0]=01111, VCOMH amplitude=VREG*1.00
	Lcd_Write_Reg(0x002D); Lcd_Write_Data(0x0006);      // NOW[2:0]=110, Gate output non-overlap period = 6 clocks
	us_Delay(2); 
	// Power on Setting
	Lcd_Write_Reg(0x0018); Lcd_Write_Data(0x0008);      // RADJ[3:0]=1000, Display frame rate 60Hz 100% 0X08 >> 135% 0XFF
	Lcd_Write_Reg(0x0019); Lcd_Write_Data(0x0001);      // OSC_EN=1, start OSC
	us_Delay(2);
	Lcd_Write_Reg(0x0001); Lcd_Write_Data(0x0000);      // DSTB=0, out deep sleep
	Lcd_Write_Reg(0x001F); Lcd_Write_Data(0x0088);      // STB=0
	Lcd_Write_Reg(0x001C); Lcd_Write_Data(0x0006);      // AP[2:0]=110, High OPAMP current (default 011)
	us_Delay(2);
	Lcd_Write_Reg(0x001F); Lcd_Write_Data(0x0080);      // DK=0
	us_Delay(2);
	Lcd_Write_Reg(0x001F); Lcd_Write_Data(0x0090);      // PON=1
	us_Delay(2);
	Lcd_Write_Reg(0x001F); Lcd_Write_Data(0x00D0);      // VCOMG=1
	us_Delay(2);
	Lcd_Write_Reg(0x0017); Lcd_Write_Data(0x0055);      // IFPF[2:0]=101, 16-bit/pixel  
	  
		// Panel Configuration
	//Lcd_Write_Reg(0x0036);      Lcd_Write_Data(0x0011);   // REV_PANEL=1, SM_PANEL=1, GS_PANEL=1, SS_PANEL=1
    //Lcd_Write_Reg(0x0029);		Lcd_Write_Data(0x0031);   // NL[5:0]=110001, 400 lines
    //Lcd_Write_Reg(0x0071);		Lcd_Write_Data(0x001A);   // RTN0
  //Gamma 2.2 Setting
	
    Lcd_Write_Reg(0x0040); Lcd_Write_Data(0x0000);	
	Lcd_Write_Reg(0x0041); Lcd_Write_Data(0x0009);
	Lcd_Write_Reg(0x0042); Lcd_Write_Data(0x0012);
	Lcd_Write_Reg(0x0043); Lcd_Write_Data(0x0004);
	Lcd_Write_Reg(0x0044); Lcd_Write_Data(0x0000);
	Lcd_Write_Reg(0x0045); Lcd_Write_Data(0x0023);      //
	Lcd_Write_Reg(0x0046); Lcd_Write_Data(0x0003);
	Lcd_Write_Reg(0x0047); Lcd_Write_Data(0x005E);      //
	Lcd_Write_Reg(0x0048); Lcd_Write_Data(0x0000);
	Lcd_Write_Reg(0x0049); Lcd_Write_Data(0x0000);
	Lcd_Write_Reg(0x004A); Lcd_Write_Data(0x0000);
	Lcd_Write_Reg(0x004B); Lcd_Write_Data(0x0000);
	Lcd_Write_Reg(0x004C); Lcd_Write_Data(0x0000);
	Lcd_Write_Reg(0x004D); Lcd_Write_Data(0x0000);
	Lcd_Write_Reg(0x004E); Lcd_Write_Data(0x0000);
	Lcd_Write_Reg(0x0057); Lcd_Write_Data(0x004F);      //
	
	 //ORIENTATION_VERTICAL 
	Lcd_Write_Reg(0x0016); Lcd_Write_Data(0x00C8);       //A8
	
		
	  
	us_Delay(2);
	Lcd_Write_Reg(0x0028); Lcd_Write_Data(0x0038);    //GON=1; DTE=1; D[1:0]=10
	us_Delay(2);
	Lcd_Write_Reg(0x0028); Lcd_Write_Data(0x003C);    //GON=1; DTE=1; D[1:0]=11	
	 us_Delay(10); 	
	//Lcd_Write_Reg(0x0022);	  
	 
}
