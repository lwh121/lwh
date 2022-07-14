/*******************************************************************************
* UART 收发例程（轮询）
*
* 本例程演示使用 USART2 收发数据
*
* UART1 作为调试串口
* 请在 debug.h 中设置  UART1 为 Debug 串口，如下
* #define DEBUG   DEBUG_UART1
*
* 开发环境：MRS
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
//上位机开启继电器标志位
u8 Relay_Flag = 0;
#define FRAME_HEAD 0x61
#define FRAME_FAIL 0x23

/*******************************************************************************
* Function Name  : USARTx_CFG
* Description    : Initializes the USART peripheral.
* 描述	：	串口初始化
* Input          : None
* Return         : None
*******************************************************************************/
void USARTx_CFG(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	//开启时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* USART2 TX-->PA2  RX-->PA3 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;           //RX，输入上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;                    // 波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;     // 数据位 8
	USART_InitStructure.USART_StopBits = USART_StopBits_1;          // 停止位 1
	USART_InitStructure.USART_Parity = USART_Parity_No;             // 无校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; //使能 RX 和 TX

	USART_Init(USART2, &USART_InitStructure);

	USART_Cmd(USART2, ENABLE);                                        //开启UART
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
    //局部变量
    int SHIDU;    //湿度 整形:0 ~ 100
    u16 nADCValue = 0;
    //中断初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	USART_Printf_Init(115200);
	//延时函数初始化
	Delay_Init();

	M_78_UserConfig();
	Relay_UserConfig();
	BEEP_UserConfig();

	//ADC初始化
	Adc_Init();

	/* LCD reset */
	//GPIO初始化
    LCD_Reset_GPIO_Init();
    //将相应的端口配置成为高电平和低电平，以此来驱动LED
    GPIO_ResetBits(GPIOA, GPIO_Pin_15);
    //延时函数100ms
    Delay_Ms(100);
    GPIO_SetBits(GPIOA, GPIO_Pin_15);
    //LCD初始化
    LCD_Init();
    //设置LCD最大亮度为50，范围0-255
    LCD_SetBrightness(50);
    //设置颜色为红色
    LCD_SetColor(0x18E3, RED);
    //设置四个点坐标，x1.y1.x2.y2
    LCD_DrawRectangle(40, 80, 200, 160, TRUE);




	while(1)
	{
	    //读取次取平均值
	    nADCValue = Get_AdcValue_Average(1,10);
	    SHIDU = (int)((4095 - nADCValue) * 100.0 / 4095) * 2;


	    printf("土壤湿度:%d\r\n", SHIDU);
	    //湿度小于50，打开继电器，浇水
	    if(SHIDU < 50 || Relay_Flag){
	        printf("继电器打开\r\n");
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
            printf("继电器关闭\r\n");
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

//对缓存区数据进行校验
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

        //串口数据放入数据缓存区
        data_buffer[count] = receive_data;

        if(receive_data == FRAME_HEAD || count > 0){
            count++;
        }else{
            count = 0;
        }


            //验证数据包的长度
            u8 len = data_buffer[1] + 4;

            if(count == len){
               count = 0;
               //帧尾
               printf("11111\r\n");
               if(data_buffer[len-1] == FRAME_FAIL){
                   printf("check is %d\r\n", Check_Sum(len-2));
                   //对数据进行异或校验
                   if(data_buffer[len-2] == Check_Sum(len-2)){

                       //如果输入open,打开继电器
                       if(strncmp(&data_buffer[2], "open", 4) == 0){
                           Relay_Flag = 1;
                           Relay_Control(1);
                       }
                       //输入close关闭继电器
                       else if(strncmp(&data_buffer[2], "close", 5) == 0){
                           Relay_Flag = 0;
                           Relay_Control(0);
                       }
                   }
               }
            }



    }
}
