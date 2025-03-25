#include <stdio.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "misc.h"
#include "USART2.h"
#include "delay.h"
#include "math.h"
#include <stdlib.h>
#include <string.h>

int flag,j=0,t,pole=1,point=0;
float Result,Result1,Result2,result[10]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
extern void TIM2_PWM_Init();//extern表示该函数在其他文件定义过，可以取代#include "xxx.h"

void Initial_UART2(unsigned long baudrate)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 
	//Tx
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//GPIO为推挽模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);    
	//Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO_Mode_IN_FLOATING浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	 
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure); 
	USART_ITConfig(USART2, USART_IT_TXE, DISABLE);    
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	
	USART_ClearFlag(USART2,USART_FLAG_TC);	
	USART_Cmd(USART2, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;//响应中断等级比串口1高
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


void USART2_IRQHandler(void) // 串口2中断服务函数，触发中断时同时处理数据，中断函数不能写太满，否则会影响实时性
{
	u8 res;
	if(USART_GetITStatus(USART2,USART_IT_RXNE)) // 中断标志
 {
   res = USART_ReceiveData(USART2);  // 串口2接收
	  if(res=='a'||res=='b'||res=='c')
    { flag=1;j=0;} //a为帧开头
    
		if(res=='-')//判断正负号
		{pole=-1;}
		
		if(flag==1&&res=='.')//记录小数位
		{ point=j;}
		
    if(flag==1&&res>='0'&&res<='9')
    {
     result[j]=res-'0'; //字符化作对应整形数字
     j++;   
    }
		
    //第一帧解析成各位数字和
    if(res=='A') 
    {
     Result=0;
		 if(point!=0) //带小数点时,point至少为1
		 {
       for(t=0;t<j;t++)
      {Result=Result+result[t]*pow(10.0,(point-t-1));} //pow(a,b)，a的b次方,ab为float位或者double类型}
		 }
		 if(point==0) //回传的是整形数字时,point=0
		 {     
			 for(t=0;t<j;t++)
      {Result=Result+result[t]*pow(10.0,(j-t-1));} //pow(a,b)，a的b次方,ab为float位或者double类型
		 }
			Result=pole*Result;
      printf("a = :%1f\r\n",Result);
      flag=0;
			j=0;
			pole=1;
			point=0;
     }
		
		//第二帧解析成各位数字和
		if(res=='B') 
    {
     Result1=0;
		 if(point!=0)
			{     
				for(t=0;t<j;t++)
      {Result1=Result1+result[t]*pow(10.0,(point-t-1));}
		  }			
		 if(point==0) //回传的是整形数字时,point=0
		  {     
			 for(t=0;t<j;t++)
      {Result1=Result1+result[t]*pow(10.0,(j-t-1));} //pow(a,b)，a的b次方,ab为float位或者double类型
		  }
			Result1=pole*Result1;
      printf("b = :%1f\r\n",Result1);
      flag=0;
			j=0;
			pole=1;
			point=0;
     }
	 
		//第三帧解析成各位数字和
		if(res=='C') 
    {
     Result2=0;
		 if(point!=0)
			{     
				for(t=0;t<j;t++)
      {Result2=Result2+result[t]*pow(10.0,(point-t-1));}
	   	}			
		 if(point==0) //回传的是整形数字时,point=0
		  {     
			 for(t=0;t<j;t++)
      {Result2=Result2+result[t]*pow(10.0,(j-t-1));} //pow(a,b)，a的b次方,ab为float位或者double类型
		  }
			Result2=pole*Result2;
      printf("c = :%1f\r\n",Result2);
      flag=0;
			j=0;
			pole=1;
			point=0;
     }
			
		 USART_ClearFlag(USART2, USART_FLAG_RXNE); //FLAG是接收寄存器非空标志位
     USART_ClearITPendingBit(USART2, USART_IT_RXNE);		//IT是接收中断标志位			
		 USART_ClearITPendingBit(USART2,USART_IT_ORE);		//溢出中断标志位		
	   USART_ClearFlag(USART2, USART_FLAG_ORE);   //溢出错误标志位
  }

}


//重定义fputc函数 ，将ch写入file文件中
//int fputc(int ch, FILE *f)//寄存器版本的fputc，库函数版本为while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);
//{      
	//USART_SendData(USART2,(uint8_t) ch);
	//while(USART_GetFlagStatus(USART2,USART_FLAG_TC==RESET));//循环发送,直到发送完毕；SR为USART的状态寄存器     
	//return ch;
//}


int fgetc(FILE *f) //函数重定向，当串口接收到数据时将这些数据传入*f指针指向的地址
{
	while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET); //如果寄存器为空则堵塞在这里
	return (int)USART_ReceiveData(USART2);
}





