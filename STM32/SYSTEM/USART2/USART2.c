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
extern void TIM2_PWM_Init();//extern��ʾ�ú����������ļ������������ȡ��#include "xxx.h"

void Initial_UART2(unsigned long baudrate)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 
	//Tx
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//GPIOΪ����ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);    
	//Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO_Mode_IN_FLOATING��������
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
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;//��Ӧ�жϵȼ��ȴ���1��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


void USART2_IRQHandler(void) // ����2�жϷ������������ж�ʱͬʱ�������ݣ��жϺ�������д̫���������Ӱ��ʵʱ��
{
	u8 res;
	if(USART_GetITStatus(USART2,USART_IT_RXNE)) // �жϱ�־
 {
   res = USART_ReceiveData(USART2);  // ����2����
	  if(res=='a'||res=='b'||res=='c')
    { flag=1;j=0;} //aΪ֡��ͷ
    
		if(res=='-')//�ж�������
		{pole=-1;}
		
		if(flag==1&&res=='.')//��¼С��λ
		{ point=j;}
		
    if(flag==1&&res>='0'&&res<='9')
    {
     result[j]=res-'0'; //�ַ�������Ӧ��������
     j++;   
    }
		
    //��һ֡�����ɸ�λ���ֺ�
    if(res=='A') 
    {
     Result=0;
		 if(point!=0) //��С����ʱ,point����Ϊ1
		 {
       for(t=0;t<j;t++)
      {Result=Result+result[t]*pow(10.0,(point-t-1));} //pow(a,b)��a��b�η�,abΪfloatλ����double����}
		 }
		 if(point==0) //�ش�������������ʱ,point=0
		 {     
			 for(t=0;t<j;t++)
      {Result=Result+result[t]*pow(10.0,(j-t-1));} //pow(a,b)��a��b�η�,abΪfloatλ����double����
		 }
			Result=pole*Result;
      printf("a = :%1f\r\n",Result);
      flag=0;
			j=0;
			pole=1;
			point=0;
     }
		
		//�ڶ�֡�����ɸ�λ���ֺ�
		if(res=='B') 
    {
     Result1=0;
		 if(point!=0)
			{     
				for(t=0;t<j;t++)
      {Result1=Result1+result[t]*pow(10.0,(point-t-1));}
		  }			
		 if(point==0) //�ش�������������ʱ,point=0
		  {     
			 for(t=0;t<j;t++)
      {Result1=Result1+result[t]*pow(10.0,(j-t-1));} //pow(a,b)��a��b�η�,abΪfloatλ����double����
		  }
			Result1=pole*Result1;
      printf("b = :%1f\r\n",Result1);
      flag=0;
			j=0;
			pole=1;
			point=0;
     }
	 
		//����֡�����ɸ�λ���ֺ�
		if(res=='C') 
    {
     Result2=0;
		 if(point!=0)
			{     
				for(t=0;t<j;t++)
      {Result2=Result2+result[t]*pow(10.0,(point-t-1));}
	   	}			
		 if(point==0) //�ش�������������ʱ,point=0
		  {     
			 for(t=0;t<j;t++)
      {Result2=Result2+result[t]*pow(10.0,(j-t-1));} //pow(a,b)��a��b�η�,abΪfloatλ����double����
		  }
			Result2=pole*Result2;
      printf("c = :%1f\r\n",Result2);
      flag=0;
			j=0;
			pole=1;
			point=0;
     }
			
		 USART_ClearFlag(USART2, USART_FLAG_RXNE); //FLAG�ǽ��ռĴ����ǿձ�־λ
     USART_ClearITPendingBit(USART2, USART_IT_RXNE);		//IT�ǽ����жϱ�־λ			
		 USART_ClearITPendingBit(USART2,USART_IT_ORE);		//����жϱ�־λ		
	   USART_ClearFlag(USART2, USART_FLAG_ORE);   //��������־λ
  }

}


//�ض���fputc���� ����chд��file�ļ���
//int fputc(int ch, FILE *f)//�Ĵ����汾��fputc���⺯���汾Ϊwhile(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);
//{      
	//USART_SendData(USART2,(uint8_t) ch);
	//while(USART_GetFlagStatus(USART2,USART_FLAG_TC==RESET));//ѭ������,ֱ��������ϣ�SRΪUSART��״̬�Ĵ���     
	//return ch;
//}


int fgetc(FILE *f) //�����ض��򣬵����ڽ��յ�����ʱ����Щ���ݴ���*fָ��ָ��ĵ�ַ
{
	while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET); //����Ĵ���Ϊ�������������
	return (int)USART_ReceiveData(USART2);
}





