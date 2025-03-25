#include "delay.h"
#include "sys.h"
#include "usart.h" 
#include "myiic.h"
#include "MS5837.h"
#include "USART2.h"
#include "math.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

void GPIO_Config(void)//GPIO初始化
{	
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_14|GPIO_Pin_15);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP; // PWM模式下推挽输出模式要改成复用输出模式，OUT改成AF
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_0|GPIO_Pin_6|GPIO_Pin_7);
}



void TIM2_PWM_Init(u16 Pulse) // 实现传感器与占空比的联动只需要设置void变成具有返回值的函数就好，PA0
	// 0,(5000*13.3/90),2*(5000*13.3/90),初始端，中间，末端
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; //需要tim.h
	TIM_OCInitTypeDef TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	TIM_TimeBaseStructure.TIM_Period=9999; //定时器输入频率为7200Hz f0=72MHz/(9999+1)
	TIM_TimeBaseStructure.TIM_Prescaler=143; //输出PWM波频率为50Hz  f=f0/(143+1)
	TIM_TimeBaseStructure.TIM_ClockDivision=0;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;  //计数模式一定要设置，否则不产生计数的话占空比无法发挥效力（因为比较计数值不变）
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure); //初始化TIM2
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse=Pulse; //占空比 Duty=Pulse/(Period+1)
	TIM_OC1Init(TIM2,&TIM_OCInitStructure); //OC1Init选择通道1,PA0口
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);//CH1预装载使能
	TIM_ARRPreloadConfig(TIM2, ENABLE); //使能TIM2在ARR上的预装载寄存器 
  TIM_CtrlPWMOutputs(TIM2,ENABLE);    //MOE主输出使能
	TIM_Cmd(TIM2,ENABLE);
}
		

void TIM3_PWM1_Init(u16 Pulse) // TIM3,CH1,PA6
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; //需要tim.h
	TIM_OCInitTypeDef TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	TIM_TimeBaseStructure.TIM_Period=9999; //定时器输入频率为7200Hz
	TIM_TimeBaseStructure.TIM_Prescaler=143; //输出PWM波频率为50Hz
	TIM_TimeBaseStructure.TIM_ClockDivision=0;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;  //计数模式一定要设置，否则不产生计数的话占空比无法发挥效力（因为比较计数值不变）
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure); //初始化TIM3
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse=Pulse; //占空比
	TIM_OC1Init(TIM3,&TIM_OCInitStructure); //OC1Init选择通道1,PA6口
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);//CH1预装载使能
	TIM_ARRPreloadConfig(TIM3, ENABLE); //使能TIM3在ARR上的预装载寄存器 
  TIM_CtrlPWMOutputs(TIM3,ENABLE);    //MOE主输出使能
	TIM_Cmd(TIM3,ENABLE);
}
void TIM3_PWM2_Init(u16 Pulse) // TIM3,CH2,PA7
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; //需要tim.h
	TIM_OCInitTypeDef TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	TIM_TimeBaseStructure.TIM_Period=9999; //定时器输入频率为7200Hz
	TIM_TimeBaseStructure.TIM_Prescaler=143; //输出PWM波频率为50Hz
	TIM_TimeBaseStructure.TIM_ClockDivision=0;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;  //计数模式一定要设置，否则不产生计数的话占空比无法发挥效力（因为比较计数值不变）
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure); //初始化TIM3
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse=Pulse; //占空比
	TIM_OC2Init(TIM3,&TIM_OCInitStructure); //OC2Init选择通道2,PA7口
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);//CH2预装载使能
	TIM_ARRPreloadConfig(TIM3, ENABLE); //使能TIM3在ARR上的预装载寄存器 
  TIM_CtrlPWMOutputs(TIM3,ENABLE);    //MOE主输出使能
	TIM_Cmd(TIM3,ENABLE);
}




int main(void)
 { 
  uint8_t data;
	int i=0,j=0,kb,km,t=0,action=0,buffer=0;
	float turn_ratio = 0, keep = 0;
	extern float Result,Result1,Result2; //调用中断服务函数计算结果，全局变量
	double B,B1,B2,M,M1,M2,f,f1,f2,R,R1,T,alpha=0,alpha1=0,alpha2=0,beta1=0,beta2=0;
  double m[3]={0,0,0},m1[3]={0,0,0},m2[3]={0,0,0},phi[2]={0,0},phi1[2]={0,0},phi2[2]={0,0},b[3]={0,0,0},b1[3]={0,0,0},b2[3]={0,0,0};
	B=100;B1=310;B2=310;  //偏置，B=0时为直行,1为尾鳍
	M=300,M1=85,M2=85; //调幅值不会影响稳定性
  f=2;f1=2.5;f2=2.5; //调频率，不完全对应实际摆动频率
  R=1;  //摆动比例
	R1=1;
  kb=5; //差分形式下会出现很多问题，稳定性和不在中心点的问题
  km=5; //kbkm会影响稳定性，收敛速度，甚至终值，所以PID输出只能调B，不能容易影响稳定性。
  T=0.001; //微分步长
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置中断优先级分组2
	delay_init();	    	 //延时函数初始化	  
	uart_init(115200);	 //串口1初始化为115200
  Initial_UART2(115200);  //串口2初始化为115200，但是优先级比串口1高
	//MS5837_init();	     //初始化MS5837
  GPIO_Config(); //一定要记得先调用GPIO口初始化函数进行初始化，再使用GPIO
	 
	 //要改成数组a[]=[角度，频率]，实现初始化阶段可调下列所有摆动状态，角度项改变PWM上下限i，频率项改变delay()，当然PWM上下限也影响整体频率
	 //用模糊控制实现角度固定，因此发送的指令不应固定为123，而是要把占空比视作变量
	 //在上位机完成模糊推理，发送if-then的指令让PWM发生改变，在数据分类阶段则完成if(USART_ReceiveData(USART2)=xxx),i=f(xxx)，从而控制左摆右摆以及幅度
	 
	//摆正到中心位置，方便装配
	 //TIM2_PWM_Init(180);
	 //TIM3_PWM1_Init(300);
	 //TIM3_PWM2_Init(300);	
	 delay_ms(2000);
	 
	 Result = 0;
	 Result1 = 0;

	while(1)
	{         
		
		B1=310-135;
		B2=310+30;
		    if (Result2 < 2)
				{
					keep = 0;  // 调节深度时保持尾鳍不动
					turn_ratio = 0;
				  buffer = 0;
				}
				
				else 
				{
					keep = -62.5;
					if(buffer!=1) // 保持上一次的动作
					{
						action = rand() % 100;  //随机生成0-99的整数，在转弯时从左转右转两个动作随机选一个
						buffer = 1;
					  if (action < 70)
					   turn_ratio = 70;  //右转弯
					  else						
					   turn_ratio = -70;  //左转弯
					}
				}
				
				if (Result1<-0.1){B1=310-205;B2=310+85;}
				if (Result1>0.1){B1=310-85;B2=310-25;}


				
				
			  //CPG algorithm
        delay_ms(5);//每隔delay_ms毫秒计算一次，跟舵机PWM占空比改变间隔平滑度有关	
        //尾鳍		M=100,B=285时在中性点30度左右摆动，注意PID控制的B正负号与舵机方向的关系，尾鳍控制B即可
				i=0;
		    //B=318+50+Result+turn_ratio;  
		    //M=100;
		    //f=3.5; //f=5时频率约为1Hz，f=3.5时频率约为0.5Hz，胸尾鳍的频率最好错开，避免共振
				B=373+Result+75;  
				M=75+Result1; //每M + 12.5尾鳍总摆角增加10°，因此75对应60°起始角
				f=2*2*3.14+Result2*3.14; //f其实是omega，即最终的周期T=2*pi/f  (0.25+Result2*0.01)*2*3.141 起始0.25Hz，每次增加0.01
		    R=1;
		
				b[i+2]=0.25*kb*kb*T*T*B-0.25*kb*kb*T*T*b[i]-kb*b[i+1]*T+kb*b[i]*T+2*b[i+1]-b[i];//状态方程
        m[i+2]=0.25*kb*km*T*T*M-0.25*kb*km*T*T*m[i]-kb*m[i+1]*T+kb*m[i]*T+2*m[i+1]-m[i];
        phi[i+1]=T*f*(((1+R)*(1+R))/(4*R)-(R*R-1)/(4*R)*sin(phi[i]))+phi[i];
					 
        alpha=b[i]+m[i]*cos(phi[i]); //输出方程
					 
        b[i]=b[i+1];b[i+1]=b[i+2]; //递推
        m[i]=m[i+1];m[i+1]=m[i+2];
        phi[i]=phi[i+1];			 
			  
				TIM2_PWM_Init(alpha+M+100); //PA0输出，加100以防小于有效占空比下限，f=5时频率约为1Hz，会有略微颤抖。PWM的值一定要在范围内，输出正弦PWM波，输出值有负有正，因此要保证最小值大于0
			  
		    //右胸鳍，胸鳍控制R;上浮[B,M]=[310-85,85]，水平B1=310-135，下潜-205
				i=0;
		    //B1=310; //和M的大小会影响同步率，最好是调好的定值
		    M1=85; //如果想要在调整摆幅之后还保持同步，那么M变化多少就需要B1以相反符号变化多少，如M+30，则B1需-30
		    f1=3.14*2*2.0; //f=5时频率约为1Hz
				R1=1+Result1; //R的输出为0.3-3.5
				b1[i+2]=0.25*kb*kb*T*T*B1-0.25*kb*kb*T*T*b1[i]-kb*b1[i+1]*T+kb*b1[i]*T+2*b1[i+1]-b1[i];//状态方程
        m1[i+2]=0.25*kb*km*T*T*M1-0.25*kb*km*T*T*m1[i]-kb*m1[i+1]*T+kb*m1[i]*T+2*m1[i+1]-m1[i];
        phi1[i+1]=T*f1*(((1+R1)*(1+R1))/(4*R1)-(R1*R1-1)/(4*R1)*sin(phi1[i]))+phi1[i];
					 
        alpha1=b1[i]+m1[i]*cos(phi1[i]); //输出方程,cos跟-cos差180度相位，这样可以控制两边胸鳍对称摆动
				beta1=b1[i]+m1[i]*(-cos(phi1[i]));
					  
        b1[i]=b1[i+1];b1[i+1]=b1[i+2]; //递推
        m1[i]=m1[i+1];m1[i+1]=m1[i+2];
        phi1[i]=phi1[i+1];			 
			  
				TIM3_PWM1_Init(alpha1+M1+50); //PA6输出，f=5时频率约为1Hz，会有略微颤抖。输出正弦PWM波，输出值有负有正，因此要保证最小值大于0
			  //TIM3_PWM2_Init(beta1+M1+50); //同步动作时，计算结果同时赋予两个舵机，因为硬件接收的result有差别，若用两个方程组可能会有不同的输出结果
			  

//				
//        //左胸鳍，胸鳍控制R;上浮[B,M]=[310-25,85]，水平B2=310+30，下潜+85
				i=0;
		    //B2=310; //和M的大小会影响同步率，最好是调好的定值
		    M2=85; //如果想要在调整摆幅之后还保持同步，那么M变化多少就需要B1以相反符号变化多少，如M+30，则B1需-30
		    f2=f1; //f=5时频率约为1Hz
				
				b2[i+2]=0.25*kb*kb*T*T*B2-0.25*kb*kb*T*T*b2[i]-kb*b2[i+1]*T+kb*b2[i]*T+2*b2[i+1]-b2[i];//状态方程
        m2[i+2]=0.25*kb*km*T*T*M2-0.25*kb*km*T*T*m2[i]-kb*m2[i+1]*T+kb*m2[i]*T+2*m2[i+1]-m2[i];
        phi2[i+1]=T*f2*(((1+R1)*(1+R1))/(4*R1)-(R1*R1-1)/(4*R1)*sin(phi2[i]))+phi2[i];
					 
        alpha2=b2[i]+m2[i]*cos(phi2[i]); //输出方程
				beta2=b2[i]+m2[i]*(-cos(phi2[i]));
					 
        b2[i]=b2[i+1];b2[i+1]=b2[i+2]; //递推
        m2[i]=m2[i+1];m2[i+1]=m2[i+2];
        phi2[i]=phi2[i+1];
				
        TIM3_PWM2_Init(beta2+M2+50);  //PA7输出，异步动作时需要这条；f=5时频率约为1Hz，会有略微颤抖。输出正弦PWM波，输出值有负有正，因此要保证最小值大于0							
				
				
		//delay_ms(10); //delay_ms()内有上限。

		//MS5837_Getdata();   //获取压力，这一句很重要！！！！ 	
		//printf("a");
		//printf("%.2f",Depth); //串口输出原始数据,Depth单位为m
		//printf("b");
		//printf("	Atmdsphere_Pressure : %u\r\n\r\n\r\n",Atmdsphere_Pressure); 
		//printf("	Pressure : %u\r\n\r\n\r\n",Pressure); //串口输出原始数据
		//printf("  P0 : %.2f  \r\n\r\n\r\n",P0);
		//printf("	Temperature : %u\r\n\r\n\r\n",Temperature); //串口输出原始数据
			

   }
}


