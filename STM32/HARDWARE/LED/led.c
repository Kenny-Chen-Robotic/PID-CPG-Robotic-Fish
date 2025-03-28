#include "led.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK miniSTM32开发板
//LED驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/2
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	   

//初始化PB5和PE5为输出口.并使能这两个口的时钟		    
//LED IO初始化
void LED_Init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 
	
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOD, ENABLE);	 //使能PA,PD端口时钟
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;				     //CH1 端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz                       //PD6
 GPIO_Init(GPIOD, &GPIO_InitStructure);					       //根据设定参数初始化GPIOD.6
 GPIO_ResetBits(GPIOD,GPIO_Pin_6);						         //PD.6 初始为低
   
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;	    		   //CH2 端口配置
 GPIO_Init(GPIOD, &GPIO_InitStructure);	  				                                             //PD7
 GPIO_ResetBits(GPIOD,GPIO_Pin_7); 		
 
 
	
	
 
                   
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;				      //极限位置上侧检测
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		                                             //PB10
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_Init(GPIOB, &GPIO_InitStructure);					 
 GPIO_ResetBits(GPIOB,GPIO_Pin_10);						            //初始化为高电平

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;             //极限位置下侧检测
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                                                //PB11
 GPIO_Init(GPIOB, &GPIO_InitStructure);	  				      
 GPIO_ResetBits(GPIOB,GPIO_Pin_11); 		                  //初始化为低电平

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;              //LED1-->PD.2 端口配置, 推挽输出           //PD1
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 	
 GPIO_Init(GPIOD, &GPIO_InitStructure);	  				      //推挽输出 ，IO口速度为50MHz
 GPIO_ResetBits(GPIOD,GPIO_Pin_1); 		


RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //使能PA,PD端口时钟
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;				 //LED0-->PA.8 端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
 GPIO_Init(GPIOA, &GPIO_InitStructure);					 //根据设定参数初始化GPIOA.8
 GPIO_ResetBits(GPIOA,GPIO_Pin_8);						 //PA.8 输出高
 
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;				 //LED0-->PA.8 端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
 GPIO_Init(GPIOA, &GPIO_InitStructure);					 //根据设定参数初始化GPIOA.8
 GPIO_ResetBits(GPIOA,GPIO_Pin_7);						 //PA.8 输出高
 


 
	
	
}
 
