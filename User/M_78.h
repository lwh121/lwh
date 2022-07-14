/*
 * M_78.h
 *
 *  Created on: 2022Äê6ÔÂ6ÈÕ
 *      Author: 13363
 */

#ifndef USER_M_78_H_
#define USER_M_78_H_

#include "debug.h"
void M_78_UserConfig(void);
void Adc_Init(void);
void LCD_Reset_GPIO_Init(void);
u16 Get_AdcValue(u8 ch);
u16 Get_AdcValue_Average(u8 ch,u8 times);

#endif /* USER_M_78_H_ */
