/*******************************************************************************
* UART �շ����̣���ѯ��
*
* ��������ʾʹ�� USART2 �շ�����
*
* UART1 ��Ϊ���Դ���
* ���� debug.h ������  UART1 Ϊ Debug ���ڣ�����
* #define DEBUG   DEBUG_UART1
*
* ����������MRS
*******************************************************************************/

/* Global define */
#define TRUE 1
#define FALSE 0
#include "debug.h"
#include "relay.h"
#include "string.h"
#include <M_78.h>
#include "lcd_st7789.h"
#include "beep.h"


void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

u8 data_buffer[100];
//��λ�������̵�����־λ
u8 Relay_Flag = 0;
#define FRAME_HEAD 0x61
#define FRAME_FAIL 0x23

/*******************************************************************************
* Function Name  : USARTx_CFG
* Description    : Initializes the USART peripheral.
* ����	��	���ڳ�ʼ��
* Input          : None
* Return         : None
*******************************************************************************/
void USARTx_CFG(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	//����ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* USART2 TX-->PA2  RX-->PA3 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;           //RX����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;                    // ������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;     // ����λ 8
	USART_InitStructure.USART_StopBits = USART_StopBits_1;          // ֹͣλ 1
	USART_InitStructure.USART_Parity = USART_Parity_No;             // ��У��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // ��Ӳ������
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; //ʹ�� RX �� TX

	USART_Init(USART2, &USART_InitStructure);

	USART_Cmd(USART2, ENABLE);                                        //����UART
}

/*******************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Return         : None
*******************************************************************************/

u8 dtLCDBuf[64];

int main(void)
{
    //�ֲ�����
    int SHIDU;    //ʪ�� ����:0 ~ 100
    u16 nADCValue = 0;
    //�жϳ�ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	USART_Printf_Init(115200);
	//��ʱ������ʼ��
	Delay_Init();

	M_78_UserConfig();
	Relay_UserConfig();
	BEEP_UserConfig();

	//ADC��ʼ��
	Adc_Init();

	/* LCD reset */
	//GPIO��ʼ��
    LCD_Reset_GPIO_Init();
    //����Ӧ�Ķ˿����ó�Ϊ�ߵ�ƽ�͵͵�ƽ���Դ�������LED
    GPIO_ResetBits(GPIOA, GPIO_Pin_15);
    //��ʱ����100ms
    Delay_Ms(100);
    GPIO_SetBits(GPIOA, GPIO_Pin_15);
    //LCD��ʼ��
    LCD_Init();
    //����LCD�������Ϊ50����Χ0-255
    LCD_SetBrightness(50);
    //������ɫΪ��ɫ
    LCD_SetColor(0x18E3, RED);
    //�����ĸ������꣬x1.y1.x2.y2
    LCD_DrawRectangle(40, 80, 200, 160, TRUE);




	while(1)
	{
	    //��ȡ��ȡƽ��ֵ
	    nADCValue = Get_AdcValue_Average(1,10);
	    SHIDU = (int)((4095 - nADCValue) * 100.0 / 4095) * 2;


	    printf("����ʪ��:%d\r\n", SHIDU);
	    //ʪ��С��50���򿪼̵�������ˮ
	    if(SHIDU < 50 || Relay_Flag){
	        printf("�̵�����\r\n");
	        Relay_Control(1);
	        LCD_Clear(BLACK);
	        BEEP_Failed(1);
            sprintf((char *)dtLCDBuf,"turangshidu:%02d ",SHIDU);
            LCD_ShowString(16, 156, 32, FALSE,dtLCDBuf);

            sprintf((char *)dtLCDBuf,"fanwei:0-100");
            LCD_ShowString(16, 104, 32, FALSE,dtLCDBuf);

            sprintf((char *)dtLCDBuf,"zhuangtai:%s","open");
            LCD_ShowString(16, 52, 32, FALSE,dtLCDBuf);
	    }
	    else {
            printf("�̵����ر�\r\n");
            Relay_Control(0);
            LCD_Clear(BLACK);
            BEEP_Failed(0);
            sprintf((char *)dtLCDBuf,"turangshidu:%02d ",SHIDU);
            LCD_ShowString(16, 156, 32, FALSE,dtLCDBuf);

            sprintf((char *)dtLCDBuf,"fanwei:0-100");
            LCD_ShowString(16, 104, 32, FALSE,dtLCDBuf);

            sprintf((char *)dtLCDBuf,"zhuangtai:%s","close");
            LCD_ShowString(0, 52, 32, FALSE,dtLCDBuf);
        }
	    Delay_Ms(1000);
	}
}

//�Ի��������ݽ���У��
u8 Check_Sum(u8 len){

    u8 checkSum = 0, i;

    for(i =0 ; i < len; i++){
        checkSum = checkSum ^ data_buffer[i];
    }

    return checkSum;
}

void USART1_IRQHandler(void)
{
    static u8 count = 0;
    u8 receive_data;

    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        receive_data = USART_ReceiveData(USART1);

        //�������ݷ������ݻ�����
        data_buffer[count] = receive_data;

        if(receive_data == FRAME_HEAD || count > 0){
            count++;
        }else{
            count = 0;
        }


            //��֤���ݰ��ĳ���
            u8 len = data_buffer[1] + 4;

            if(count == len){
               count = 0;
               //֡β
               printf("11111\r\n");
               if(data_buffer[len-1] == FRAME_FAIL){
                   printf("check is %d\r\n", Check_Sum(len-2));
                   //�����ݽ������У��
                   if(data_buffer[len-2] == Check_Sum(len-2)){

                       //�������open,�򿪼̵���
                       if(strncmp(&data_buffer[2], "open", 4) == 0){
                           Relay_Flag = 1;
                           Relay_Control(1);
                       }
                       //����close�رռ̵���
                       else if(strncmp(&data_buffer[2], "close", 5) == 0){
                           Relay_Flag = 0;
                           Relay_Control(0);
                       }
                   }
               }
            }



    }
}
