
#include "delay.h"
#include "sys.h"
#include "usart.h" 
#include "myiic.h"
#include "MS5837.h"


 int main(void)
 { 

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// �����ж����ȼ�����2
	delay_init();	    	 //��ʱ������ʼ��	  
	uart_init(115200);	 //���ڳ�ʼ��Ϊ115200
	MS5837_init();	     //��ʼ��MS5837
	 
	while(1)
	{
		delay_ms(500); //delay_ms()�������ޣ�Ҫ�ü���delay�ӳ�ʱ�䣬Ŀǰ�����4���ӡ��ϴ��ٶ�Ҫ�죬��ݮ�����ö�ȡ�������С�

		MS5837_Getdata();   //��ȡѹ��
			
		//printf("	Atmdsphere_Pressure : %u\r\n\r\n\r\n",Atmdsphere_Pressure); 
		//printf("	Pressure : %u\r\n\r\n\r\n",Pressure); //�������ԭʼ����
		//printf("  P0 : %.2f  \r\n\r\n\r\n",P0);
		//printf("	Temperature : %u\r\n\r\n\r\n",Temperature); //�������ԭʼ����
		printf("%.2f",Depth); //�������ԭʼ����
		 
			

   }
	}

