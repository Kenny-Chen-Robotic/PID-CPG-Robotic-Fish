
#include "delay.h"
#include "sys.h"
#include "usart.h" 
#include "myiic.h"
#include "MS5837.h"


 int main(void)
 { 

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置中断优先级分组2
	delay_init();	    	 //延时函数初始化	  
	uart_init(115200);	 //串口初始化为115200
	MS5837_init();	     //初始化MS5837
	 
	while(1)
	{
		delay_ms(500); //delay_ms()内有上限，要用几个delay延长时间，目前大概是4秒钟。上传速度要快，树莓派设置读取间隔大就行。

		MS5837_Getdata();   //获取压力
			
		//printf("	Atmdsphere_Pressure : %u\r\n\r\n\r\n",Atmdsphere_Pressure); 
		//printf("	Pressure : %u\r\n\r\n\r\n",Pressure); //串口输出原始数据
		//printf("  P0 : %.2f  \r\n\r\n\r\n",P0);
		//printf("	Temperature : %u\r\n\r\n\r\n",Temperature); //串口输出原始数据
		printf("%.2f",Depth); //串口输出原始数据
		 
			

   }
	}

