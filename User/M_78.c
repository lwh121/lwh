/*
 * M_78.c
 *
 *  Created on: 2022��6��6��
 *      Author: 13363
 */
#include <M_78.h>

void M_78_UserConfig(void){

    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;//PB15
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
    GPIO_Init(GPIOB,&GPIO_InitStructure);//��ʼ������

    //GPIO_SetBits(Relay_PROT,Relay);//��ʼ״̬����
}
void Adc_Init(void){

    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_ADC1,ENABLE);//ADC1

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);//72/6=12  ADC���ܳ���14Mhz ����Ҫ��Ƶ

    //PA0
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;//ģ������
    GPIO_Init(GPIOA,&GPIO_InitStructure);

    //ADC
    ADC_DeInit(ADC1);//��λADC1����
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;//��ͨ��
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//����ת��
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//����ģʽ
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//ʹ�������������ʹ���жϴ���
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//12λ����λ�Ҷ���
    ADC_InitStructure.ADC_NbrOfChannel = 1;//ͨ������1
    ADC_Init(ADC1,&ADC_InitStructure);

    ADC_Cmd(ADC1,ENABLE);//ȫ��ʹ��ADC

    ADC_ResetCalibration(ADC1);//ʹ�ܸ�λУ׼
    while(ADC_GetResetCalibrationStatus(ADC1));//�ȴ���λУ׼����

    ADC_StartCalibration(ADC1);//����ADУ׼
    while(ADC_GetCalibrationStatus(ADC1));//�ȴ�ADУ׼����
}
//��ȡADCֵ
//ͨ��ch 0-3
u16 Get_AdcValue(u8 ch){

    ADC_RegularChannelConfig(ADC1,ch,1,ADC_SampleTime_239Cycles5);//ADC�Ĺ�����:ͨ�� һ�����У�����ʱ�� ����ʱ��Ϊ239.5����
    ADC_SoftwareStartConvCmd(ADC1,ENABLE);//ADC1���ת������

    while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));//�ȴ�ADCת������

    return ADC_GetConversionValue(ADC1);
}
//����õ�����ȥƽ��ֵ
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


