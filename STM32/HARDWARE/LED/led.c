#include "led.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK mini�SSTM32������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	   

//��ʼ��PB5��PE5Ϊ�����.��ʹ���������ڵ�ʱ��		    
//LED IO��ʼ��
void LED_Init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 
	
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOD, ENABLE);	 //ʹ��PA,PD�˿�ʱ��
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;				     //CH1 �˿�����
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz                       //PD6
 GPIO_Init(GPIOD, &GPIO_InitStructure);					       //�����趨������ʼ��GPIOD.6
 GPIO_ResetBits(GPIOD,GPIO_Pin_6);						         //PD.6 ��ʼΪ��
   
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;	    		   //CH2 �˿�����
 GPIO_Init(GPIOD, &GPIO_InitStructure);	  				                                             //PD7
 GPIO_ResetBits(GPIOD,GPIO_Pin_7); 		
 
 
	
	
 
                   
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;				      //����λ���ϲ���
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		                                             //PB10
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_Init(GPIOB, &GPIO_InitStructure);					 
 GPIO_ResetBits(GPIOB,GPIO_Pin_10);						            //��ʼ��Ϊ�ߵ�ƽ

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;             //����λ���²���
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                                                //PB11
 GPIO_Init(GPIOB, &GPIO_InitStructure);	  				      
 GPIO_ResetBits(GPIOB,GPIO_Pin_11); 		                  //��ʼ��Ϊ�͵�ƽ

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;              //LED1-->PD.2 �˿�����, �������           //PD1
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 	
 GPIO_Init(GPIOD, &GPIO_InitStructure);	  				      //������� ��IO���ٶ�Ϊ50MHz
 GPIO_ResetBits(GPIOD,GPIO_Pin_1); 		


RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //ʹ��PA,PD�˿�ʱ��
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;				 //LED0-->PA.8 �˿�����
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
 GPIO_Init(GPIOA, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOA.8
 GPIO_ResetBits(GPIOA,GPIO_Pin_8);						 //PA.8 �����
 
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;				 //LED0-->PA.8 �˿�����
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
 GPIO_Init(GPIOA, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOA.8
 GPIO_ResetBits(GPIOA,GPIO_Pin_7);						 //PA.8 �����
 


 
	
	
}
 
