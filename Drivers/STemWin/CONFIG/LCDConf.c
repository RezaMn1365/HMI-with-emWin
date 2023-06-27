/*********************************************************************
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2017  SEGGER Microcontroller GmbH & Co. KG       *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

** emWin V5.44 - Graphical user interface for embedded applications **
All  Intellectual Property rights  in the Software belongs to  SEGGER.
emWin is protected by  international copyright laws.  Knowledge of the
source code may not be used to write a similar product.  This file may
only be used in accordance with the following terms:

The  software has  been licensed  to STMicroelectronics International
N.V. a Dutch company with a Swiss branch and its headquarters in Plan-
les-Ouates, Geneva, 39 Chemin du Champ des Filles, Switzerland for the
purposes of creating libraries for ARM Cortex-M-based 32-bit microcon_
troller products commercialized by Licensee only, sublicensed and dis_
tributed under the terms and conditions of the End User License Agree_
ment supplied by STMicroelectronics International N.V.
Full source code is available at: www.segger.com

We appreciate your understanding and fairness.
----------------------------------------------------------------------
File        : LCDConf.c
Purpose     : Display controller configuration (single layer)
---------------------------END-OF-HEADER------------------------------
*/

/**
  ******************************************************************************
  * @file    LCDConf_stm32373c_eval.c
  * @author  MCD Application Team
  * @brief   Driver for STM32373C-EVAL RevB board LCD
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

#include "GUI.h"
#include "GUIDRV_FlexColor.h"
#include "main.h"
#include "HX8352B-F4-PAR.h"
#include "GUITDRV_ADS7846.h"
#include "XPT2046-REZ.h"

/*********************************************************************
*
*       Layer configuration (to be modified)
*
**********************************************************************
*/

//
// Physical display size
//
#define XSIZE_PHYS  240
#define YSIZE_PHYS  400

/*********************************************************************
*
*       Configuration checking
*
**********************************************************************
*/
#ifndef   VXSIZE_PHYS
#define VXSIZE_PHYS XSIZE_PHYS
#endif
#ifndef   VYSIZE_PHYS
#define VYSIZE_PHYS YSIZE_PHYS
#endif
#ifndef   XSIZE_PHYS
#error Physical X size of display is not defined!
#endif
#ifndef   YSIZE_PHYS
#error Physical Y size of display is not defined!
#endif
#ifndef   GUICC_565
#error Color conversion not defined!
#endif
#ifndef   GUIDRV_FLEXCOLOR
#error No display driver defined!
#endif


/*********************************************************************
*
*       Defines, sfrs
*
**********************************************************************
*/
//
// COG interface register addr.
//

typedef struct
{
  __IO uint16_t REG;
  __IO uint16_t RAM;

} LCD_CONTROLLER_TypeDef;

/*********************************************************************
*
*       Local functions
*
**********************************************************************
*/
static void LCD_LL_Init(void);


/********************************************************************
*
*       LcdWriteReg
*
* Function description:
*   Sets display register
*/
static void LcdWriteReg(U16 Data)
{
	Lcd_Write_Reg(Data);
}

/********************************************************************
*
*       LcdWriteData
*
* Function description:
*   Writes a value to a display register
*/
static void LcdWriteData(U16 Data)
{
	Lcd_Write_Data(Data);
	
}

/********************************************************************
*
*       LcdWriteDataMultiple
*
* Function description:
*   Writes multiple values to a display register.
*/
static void LcdWriteDataMultiple(U16 *pData, int NumItems)
{
	while (NumItems--)
	{
		
		Lcd_Write_Data(*pData++);
	}
 
}

/********************************************************************
*
*       LcdReadDataMultiple
*
* Function description:
*   Reads multiple values from a display register.
*/
static void LcdReadDataMultiple(U16 *pData, int NumItems)
{
  while (NumItems--)
  {
	 *pData++  = Lcd_Read_Data();    
  }
}

/*********************************************************************
*
*       Public functions
*
**********************************************************************
*/

/**
  * @brief  Initializes the LCD.
  * @param  None
  * @retval LCD state
  */
static void LCD_LL_Init(void)
{
	Lcd_Init();
  
}

/*********************************************************************
*
*       LCD_X_Config
*
* Function description:
*   Called during the initialization process in order to set up the
*   display driver configuration.
*
*/
void LCD_X_Config(void)
{
	GUI_DEVICE *pDevice;
	CONFIG_FLEXCOLOR Config = { 0 };
	GUI_PORT_API PortAPI = { 0 };
	//
	// Set display driver and color conversion
	//
	pDevice = GUI_DEVICE_CreateAndLink(GUIDRV_FLEXCOLOR, GUICC_565, 0, 0);
	//
	// Display driver configuration, required for Lin-driver
	//
	LCD_SetSizeEx(0, XSIZE_PHYS, YSIZE_PHYS);
	LCD_SetVSizeEx(0, VXSIZE_PHYS, VYSIZE_PHYS);
	//
	// Orientation
	//
	Config.Orientation = GUI_SWAP_XY | GUI_MIRROR_Y;
	//Config.NumDummyReads = 1;

	GUIDRV_FlexColor_Config(pDevice, &Config);
	//
	// Set controller and operation mode
	//
	PortAPI.pfWrite16_A0  = LcdWriteReg;
	PortAPI.pfWrite16_A1  = LcdWriteData;
	PortAPI.pfWriteM16_A1 = LcdWriteDataMultiple;
	PortAPI.pfReadM16_A1  = LcdReadDataMultiple;

	// Find the current LCD and initialize GUIDRV
	GUIDRV_FlexColor_SetReadFunc66715_B16(pDevice, GUIDRV_FLEXCOLOR_READ_FUNC_III);
	GUIDRV_FlexColor_SetFunc(pDevice, &PortAPI, GUIDRV_FLEXCOLOR_F66715, GUIDRV_FLEXCOLOR_M16C0B16);
	

	
/*********************************************************************
*
*       TOUCH_X_Config
*
* Function description:
*   Called during the initialization process in order to set up the
*   display driver configuration.
*
*/
	
//void TOUCH_X_Init(void);
//void TOUCH_X_Init(void) {
	//GUITDRV_ADS7846_CONFIG touch_config = { 0 };
	
	
	
	
	GUITDRV_ADS7846_CONFIG touch_config = { 0 };
	touch_config.pfSendCmd = &SendCmd;
	touch_config.pfGetBusy = &GetBusy;
	touch_config.pfGetResult = &GetResult;
	touch_config.pfSetCS = &SetCS;
	touch_config.Orientation = GUI_SWAP_XY;//| GUI_MIRROR_X;
	touch_config.xLog0 = 0;
	touch_config.xLog1 = 239;
	touch_config.xPhys0 = 0X0080;
	touch_config.xPhys1 = 0X0777;
	touch_config.yLog0 = 0;
	touch_config.yLog1 = 399;
	touch_config.yPhys1 = 0X0064; 
	touch_config.yPhys0 = 0X075C;
	touch_config.pfGetPENIRQ = &GetPENIRQ;
	
	
	GUITDRV_ADS7846_Config(&touch_config);
  
}

/*********************************************************************
*
*       LCD_X_DisplayDriver
*
* Function description:
*   This function is called by the display driver for several purposes.
*   To support the according task the routine needs to be adapted to
*   the display controller. Please note that the commands marked with
*   'optional' are not cogently required and should only be adapted if
*   the display controller supports these features.
*
* Parameter:
*   LayerIndex - Index of layer to be configured
*   Cmd        - Please refer to the details in the switch statement below
*   pData      - Pointer to a LCD_X_DATA structure
*
* Return Value:
*   < -1 - Error
*     -1 - Command not handled
*      0 - Ok
*/
int LCD_X_DisplayDriver(unsigned LayerIndex, unsigned Cmd, void *pData)
{
  int r;
  (void) LayerIndex;
  (void) pData;

  switch (Cmd)
  {
    case LCD_X_INITCONTROLLER:
    {

      LCD_LL_Init();

      return 0;
    }
    default:
      r = -1;
  }
  return r;
}


/*************************** End of file ****************************/

