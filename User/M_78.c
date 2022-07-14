/*
 * M_78.c
 *
 *  Created on: 2022年6月6日
 *      Author: 13363
 */
#include <M_78.h>

void M_78_UserConfig(void){

    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;//PB15
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//推挽输入
    GPIO_Init(GPIOB,&GPIO_InitStructure);//初始化引脚

    //GPIO_SetBits(Relay_PROT,Relay);//起始状态拉高
}
void Adc_Init(void){

    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_ADC1,ENABLE);//ADC1

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);//72/6=12  ADC不能超过14Mhz 故需要分频

    //PA0
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;//模拟输入
    GPIO_Init(GPIOA,&GPIO_InitStructure);

    //ADC
    ADC_DeInit(ADC1);//复位ADC1引脚
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;//单通道
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//连续转换
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//使用软件触发，不使用中断触发
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//12位数据位右对齐
    ADC_InitStructure.ADC_NbrOfChannel = 1;//通道数量1
    ADC_Init(ADC1,&ADC_InitStructure);

    ADC_Cmd(ADC1,ENABLE);//全局使能ADC

    ADC_ResetCalibration(ADC1);//使能复位校准
    while(ADC_GetResetCalibrationStatus(ADC1));//等待复位校准结束

    ADC_StartCalibration(ADC1);//开启AD校准
    while(ADC_GetCalibrationStatus(ADC1));//等待AD校准结束
}
//获取ADC值
//通道ch 0-3
u16 Get_AdcValue(u8 ch){

    ADC_RegularChannelConfig(ADC1,ch,1,ADC_SampleTime_239Cycles5);//ADC的规则组:通道 一个序列，采样时间 采样时间为239.5周期
    ADC_SoftwareStartConvCmd(ADC1,ENABLE);//ADC1软件转换启动

    while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));//等待ADC转换结束

    return ADC_GetConversionValue(ADC1);
}
//对求得的数据去平均值
u16 Get_AdcValue_Average(u8 ch,u8 times){

    u32 temp = 0;
    u8 i;
    for(i=0;i<times;i++){

        temp+=Get_AdcValue(ch);
        Delay_Ms(5);
    }
    return temp/times;
}

void LCD_Reset_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_SetBits(GPIOA, GPIO_Pin_15);
}


