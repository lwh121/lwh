/*
 * relay.h
 *
 *  Created on: 2022Äê6ÔÂ6ÈÕ
 *      Author: 13363
 */

#ifndef USER_RELAY_H_
#define USER_RELAY_H_

#include "debug.h"

#define Relay GPIO_Pin_0
#define Relay_PROT GPIOB

void Relay_UserConfig(void);
void Relay_Control(u8 open);



#endif /* USER_RELAY_H_ */
