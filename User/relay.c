/*
 * relay.c
 *
 *  Created on: 2022年6月6日
 *      Author: 13363
 */
#include "relay.h"

void Relay_UserConfig(void){

    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

    GPIO_InitStructure.GPIO_Pin = Relay;//PB15
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出
    GPIO_Init(Relay_PROT,&GPIO_InitStructure);//初始化引脚

    GPIO_SetBits(Relay_PROT,Relay);//起始状态拉高
}

void Relay_Control(u8 open){

    if(open)
    {
        GPIO_ResetBits(Relay_PROT,Relay);//打开
    }
    else
    {
        GPIO_SetBits(Relay_PROT,Relay);//关闭
    }
}


