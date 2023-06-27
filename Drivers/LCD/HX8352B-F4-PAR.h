#include "stdint.h"

void LCD_DATA_GPIO_IN();
void LCD_DATA_GPIO_OUT();
void delay_us(uint16_t US);
void LCDRST(void);
void Lcd_Write_Reg(uint16_t address);
void Lcd_Write_Data(uint16_t data);
uint16_t Lcd_Read_Data();
void Lcd_Init();