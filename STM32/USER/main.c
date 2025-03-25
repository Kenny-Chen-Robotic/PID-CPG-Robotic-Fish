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

void GPIO_Config(void)//GPIO��ʼ��
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
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP; // PWMģʽ���������ģʽҪ�ĳɸ������ģʽ��OUT�ĳ�AF
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_0|GPIO_Pin_6|GPIO_Pin_7);
}



void TIM2_PWM_Init(u16 Pulse) // ʵ�ִ�������ռ�ձȵ�����ֻ��Ҫ����void��ɾ��з���ֵ�ĺ����ͺã�PA0
	// 0,(5000*13.3/90),2*(5000*13.3/90),��ʼ�ˣ��м䣬ĩ��
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; //��Ҫtim.h
	TIM_OCInitTypeDef TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	TIM_TimeBaseStructure.TIM_Period=9999; //��ʱ������Ƶ��Ϊ7200Hz f0=72MHz/(9999+1)
	TIM_TimeBaseStructure.TIM_Prescaler=143; //���PWM��Ƶ��Ϊ50Hz  f=f0/(143+1)
	TIM_TimeBaseStructure.TIM_ClockDivision=0;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;  //����ģʽһ��Ҫ���ã����򲻲��������Ļ�ռ�ձ��޷�����Ч������Ϊ�Ƚϼ���ֵ���䣩
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure); //��ʼ��TIM2
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse=Pulse; //ռ�ձ� Duty=Pulse/(Period+1)
	TIM_OC1Init(TIM2,&TIM_OCInitStructure); //OC1Initѡ��ͨ��1,PA0��
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);//CH1Ԥװ��ʹ��
	TIM_ARRPreloadConfig(TIM2, ENABLE); //ʹ��TIM2��ARR�ϵ�Ԥװ�ؼĴ��� 
  TIM_CtrlPWMOutputs(TIM2,ENABLE);    //MOE�����ʹ��
	TIM_Cmd(TIM2,ENABLE);
}
		

void TIM3_PWM1_Init(u16 Pulse) // TIM3,CH1,PA6
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; //��Ҫtim.h
	TIM_OCInitTypeDef TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	TIM_TimeBaseStructure.TIM_Period=9999; //��ʱ������Ƶ��Ϊ7200Hz
	TIM_TimeBaseStructure.TIM_Prescaler=143; //���PWM��Ƶ��Ϊ50Hz
	TIM_TimeBaseStructure.TIM_ClockDivision=0;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;  //����ģʽһ��Ҫ���ã����򲻲��������Ļ�ռ�ձ��޷�����Ч������Ϊ�Ƚϼ���ֵ���䣩
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure); //��ʼ��TIM3
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse=Pulse; //ռ�ձ�
	TIM_OC1Init(TIM3,&TIM_OCInitStructure); //OC1Initѡ��ͨ��1,PA6��
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);//CH1Ԥװ��ʹ��
	TIM_ARRPreloadConfig(TIM3, ENABLE); //ʹ��TIM3��ARR�ϵ�Ԥװ�ؼĴ��� 
  TIM_CtrlPWMOutputs(TIM3,ENABLE);    //MOE�����ʹ��
	TIM_Cmd(TIM3,ENABLE);
}
void TIM3_PWM2_Init(u16 Pulse) // TIM3,CH2,PA7
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; //��Ҫtim.h
	TIM_OCInitTypeDef TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	TIM_TimeBaseStructure.TIM_Period=9999; //��ʱ������Ƶ��Ϊ7200Hz
	TIM_TimeBaseStructure.TIM_Prescaler=143; //���PWM��Ƶ��Ϊ50Hz
	TIM_TimeBaseStructure.TIM_ClockDivision=0;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;  //����ģʽһ��Ҫ���ã����򲻲��������Ļ�ռ�ձ��޷�����Ч������Ϊ�Ƚϼ���ֵ���䣩
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure); //��ʼ��TIM3
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse=Pulse; //ռ�ձ�
	TIM_OC2Init(TIM3,&TIM_OCInitStructure); //OC2Initѡ��ͨ��2,PA7��
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);//CH2Ԥװ��ʹ��
	TIM_ARRPreloadConfig(TIM3, ENABLE); //ʹ��TIM3��ARR�ϵ�Ԥװ�ؼĴ��� 
  TIM_CtrlPWMOutputs(TIM3,ENABLE);    //MOE�����ʹ��
	TIM_Cmd(TIM3,ENABLE);
}




int main(void)
 { 
  uint8_t data;
	int i=0,j=0,kb,km,t=0,action=0,buffer=0;
	float turn_ratio = 0, keep = 0;
	extern float Result,Result1,Result2; //�����жϷ�������������ȫ�ֱ���
	double B,B1,B2,M,M1,M2,f,f1,f2,R,R1,T,alpha=0,alpha1=0,alpha2=0,beta1=0,beta2=0;
  double m[3]={0,0,0},m1[3]={0,0,0},m2[3]={0,0,0},phi[2]={0,0},phi1[2]={0,0},phi2[2]={0,0},b[3]={0,0,0},b1[3]={0,0,0},b2[3]={0,0,0};
	B=100;B1=310;B2=310;  //ƫ�ã�B=0ʱΪֱ��,1Ϊβ��
	M=300,M1=85,M2=85; //����ֵ����Ӱ���ȶ���
  f=2;f1=2.5;f2=2.5; //��Ƶ�ʣ�����ȫ��Ӧʵ�ʰڶ�Ƶ��
  R=1;  //�ڶ�����
	R1=1;
  kb=5; //�����ʽ�»���ֺܶ����⣬�ȶ��ԺͲ������ĵ������
  km=5; //kbkm��Ӱ���ȶ��ԣ������ٶȣ�������ֵ������PID���ֻ�ܵ�B����������Ӱ���ȶ��ԡ�
  T=0.001; //΢�ֲ���
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// �����ж����ȼ�����2
	delay_init();	    	 //��ʱ������ʼ��	  
	uart_init(115200);	 //����1��ʼ��Ϊ115200
  Initial_UART2(115200);  //����2��ʼ��Ϊ115200���������ȼ��ȴ���1��
	//MS5837_init();	     //��ʼ��MS5837
  GPIO_Config(); //һ��Ҫ�ǵ��ȵ���GPIO�ڳ�ʼ���������г�ʼ������ʹ��GPIO
	 
	 //Ҫ�ĳ�����a[]=[�Ƕȣ�Ƶ��]��ʵ�ֳ�ʼ���׶οɵ��������аڶ�״̬���Ƕ���ı�PWM������i��Ƶ����ı�delay()����ȻPWM������ҲӰ������Ƶ��
	 //��ģ������ʵ�ֽǶȹ̶�����˷��͵�ָ�Ӧ�̶�Ϊ123������Ҫ��ռ�ձ���������
	 //����λ�����ģ����������if-then��ָ����PWM�����ı䣬�����ݷ���׶������if(USART_ReceiveData(USART2)=xxx),i=f(xxx)���Ӷ���������Ұ��Լ�����
	 
	//����������λ�ã�����װ��
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
					keep = 0;  // �������ʱ����β������
					turn_ratio = 0;
				  buffer = 0;
				}
				
				else 
				{
					keep = -62.5;
					if(buffer!=1) // ������һ�εĶ���
					{
						action = rand() % 100;  //�������0-99����������ת��ʱ����ת��ת�����������ѡһ��
						buffer = 1;
					  if (action < 70)
					   turn_ratio = 70;  //��ת��
					  else						
					   turn_ratio = -70;  //��ת��
					}
				}
				
				if (Result1<-0.1){B1=310-205;B2=310+85;}
				if (Result1>0.1){B1=310-85;B2=310-25;}


				
				
			  //CPG algorithm
        delay_ms(5);//ÿ��delay_ms�������һ�Σ������PWMռ�ձȸı���ƽ�����й�	
        //β��		M=100,B=285ʱ�����Ե�30�����Ұڶ���ע��PID���Ƶ�B��������������Ĺ�ϵ��β������B����
				i=0;
		    //B=318+50+Result+turn_ratio;  
		    //M=100;
		    //f=3.5; //f=5ʱƵ��ԼΪ1Hz��f=3.5ʱƵ��ԼΪ0.5Hz����β����Ƶ����ô������⹲��
				B=373+Result+75;  
				M=75+Result1; //ÿM + 12.5β���ܰڽ�����10�㣬���75��Ӧ60����ʼ��
				f=2*2*3.14+Result2*3.14; //f��ʵ��omega�������յ�����T=2*pi/f  (0.25+Result2*0.01)*2*3.141 ��ʼ0.25Hz��ÿ������0.01
		    R=1;
		
				b[i+2]=0.25*kb*kb*T*T*B-0.25*kb*kb*T*T*b[i]-kb*b[i+1]*T+kb*b[i]*T+2*b[i+1]-b[i];//״̬����
        m[i+2]=0.25*kb*km*T*T*M-0.25*kb*km*T*T*m[i]-kb*m[i+1]*T+kb*m[i]*T+2*m[i+1]-m[i];
        phi[i+1]=T*f*(((1+R)*(1+R))/(4*R)-(R*R-1)/(4*R)*sin(phi[i]))+phi[i];
					 
        alpha=b[i]+m[i]*cos(phi[i]); //�������
					 
        b[i]=b[i+1];b[i+1]=b[i+2]; //����
        m[i]=m[i+1];m[i+1]=m[i+2];
        phi[i]=phi[i+1];			 
			  
				TIM2_PWM_Init(alpha+M+100); //PA0�������100�Է�С����Чռ�ձ����ޣ�f=5ʱƵ��ԼΪ1Hz��������΢������PWM��ֵһ��Ҫ�ڷ�Χ�ڣ��������PWM�������ֵ�и����������Ҫ��֤��Сֵ����0
			  
		    //����������������R;�ϸ�[B,M]=[310-85,85]��ˮƽB1=310-135����Ǳ-205
				i=0;
		    //B1=310; //��M�Ĵ�С��Ӱ��ͬ���ʣ�����ǵ��õĶ�ֵ
		    M1=85; //�����Ҫ�ڵ����ڷ�֮�󻹱���ͬ������ôM�仯���پ���ҪB1���෴���ű仯���٣���M+30����B1��-30
		    f1=3.14*2*2.0; //f=5ʱƵ��ԼΪ1Hz
				R1=1+Result1; //R�����Ϊ0.3-3.5
				b1[i+2]=0.25*kb*kb*T*T*B1-0.25*kb*kb*T*T*b1[i]-kb*b1[i+1]*T+kb*b1[i]*T+2*b1[i+1]-b1[i];//״̬����
        m1[i+2]=0.25*kb*km*T*T*M1-0.25*kb*km*T*T*m1[i]-kb*m1[i+1]*T+kb*m1[i]*T+2*m1[i+1]-m1[i];
        phi1[i+1]=T*f1*(((1+R1)*(1+R1))/(4*R1)-(R1*R1-1)/(4*R1)*sin(phi1[i]))+phi1[i];
					 
        alpha1=b1[i]+m1[i]*cos(phi1[i]); //�������,cos��-cos��180����λ���������Կ������������Գưڶ�
				beta1=b1[i]+m1[i]*(-cos(phi1[i]));
					  
        b1[i]=b1[i+1];b1[i+1]=b1[i+2]; //����
        m1[i]=m1[i+1];m1[i+1]=m1[i+2];
        phi1[i]=phi1[i+1];			 
			  
				TIM3_PWM1_Init(alpha1+M1+50); //PA6�����f=5ʱƵ��ԼΪ1Hz��������΢�������������PWM�������ֵ�и����������Ҫ��֤��Сֵ����0
			  //TIM3_PWM2_Init(beta1+M1+50); //ͬ������ʱ��������ͬʱ���������������ΪӲ�����յ�result�в������������������ܻ��в�ͬ��������
			  

//				
//        //����������������R;�ϸ�[B,M]=[310-25,85]��ˮƽB2=310+30����Ǳ+85
				i=0;
		    //B2=310; //��M�Ĵ�С��Ӱ��ͬ���ʣ�����ǵ��õĶ�ֵ
		    M2=85; //�����Ҫ�ڵ����ڷ�֮�󻹱���ͬ������ôM�仯���پ���ҪB1���෴���ű仯���٣���M+30����B1��-30
		    f2=f1; //f=5ʱƵ��ԼΪ1Hz
				
				b2[i+2]=0.25*kb*kb*T*T*B2-0.25*kb*kb*T*T*b2[i]-kb*b2[i+1]*T+kb*b2[i]*T+2*b2[i+1]-b2[i];//״̬����
        m2[i+2]=0.25*kb*km*T*T*M2-0.25*kb*km*T*T*m2[i]-kb*m2[i+1]*T+kb*m2[i]*T+2*m2[i+1]-m2[i];
        phi2[i+1]=T*f2*(((1+R1)*(1+R1))/(4*R1)-(R1*R1-1)/(4*R1)*sin(phi2[i]))+phi2[i];
					 
        alpha2=b2[i]+m2[i]*cos(phi2[i]); //�������
				beta2=b2[i]+m2[i]*(-cos(phi2[i]));
					 
        b2[i]=b2[i+1];b2[i+1]=b2[i+2]; //����
        m2[i]=m2[i+1];m2[i+1]=m2[i+2];
        phi2[i]=phi2[i+1];
				
        TIM3_PWM2_Init(beta2+M2+50);  //PA7������첽����ʱ��Ҫ������f=5ʱƵ��ԼΪ1Hz��������΢�������������PWM�������ֵ�и����������Ҫ��֤��Сֵ����0							
				
				
		//delay_ms(10); //delay_ms()�������ޡ�

		//MS5837_Getdata();   //��ȡѹ������һ�����Ҫ�������� 	
		//printf("a");
		//printf("%.2f",Depth); //�������ԭʼ����,Depth��λΪm
		//printf("b");
		//printf("	Atmdsphere_Pressure : %u\r\n\r\n\r\n",Atmdsphere_Pressure); 
		//printf("	Pressure : %u\r\n\r\n\r\n",Pressure); //�������ԭʼ����
		//printf("  P0 : %.2f  \r\n\r\n\r\n",P0);
		//printf("	Temperature : %u\r\n\r\n\r\n",Temperature); //�������ԭʼ����
			

   }
}


