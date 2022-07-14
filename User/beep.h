/*
 * beep.h
 *
 *  Created on: 2022Äê6ÔÂ7ÈÕ
 *      Author: 13363
 */

#ifndef USER_BEEP_H_
#define USER_BEEP_H_

#include "debug.h"

#define BEEP GPIO_Pin_3
#define BEEP_PROT GPIOD

void BEEP_UserConfig(void);
void BEEP_Failed(u8);



#endif /* USER_BEEP_H_ */
