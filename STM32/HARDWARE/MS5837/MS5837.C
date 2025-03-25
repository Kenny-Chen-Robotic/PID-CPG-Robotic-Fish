#include "MS5837.h"
#include "myiic.h"
#include "usart.h"
#include "delay.h"

/*
C1 压力灵敏度 SENS|T1
C2  压力补偿  OFF|T1
C3	温度压力灵敏度系数 TCS
C4	温度系数的压力补偿 TCO
C5	参考温度 T|REF
C6 	温度系数的温度 TEMPSENS
*/
uint32_t  Cal_C[7];	        //用于存放PROM中的6组数据1-6

double OFF_;
float Aux;
/*
dT 实际和参考温度之间的差异
TEMP 实际温度	
*/
signed int dT,TEMP;
/*
OFF 实际温度补偿
SENS 实际温度灵敏度
*/
uint64_t SENS;
uint32_t D1_Pres,D2_Temp;	// 数字压力值,数字温度值
uint32_t TEMP2,T2,OFF2,SENS2;	//温度校验值

uint32_t Pressure,Temperature;				//气压
uint32_t Atmdsphere_Pressure;//大气压
uint32_t C5;
double P0;
uint32_t P1;
double Depth; //深度的精度不够

/*******************************************************************************
  * @函数名称	MS583730BA_RESET
  * @函数说明   复位MS5611
  * @输入参数   无
  * @输出参数   无
  * @返回参数   无
*******************************************************************************/
void MS583703BA_RESET(void)
{
		IIC_Start();
		IIC_Send_Byte(0xEC);//CSB接地，主机地址：0XEE，否则 0X77
	  IIC_Wait_Ack();
    IIC_Send_Byte(0x1E);//发送复位命令
	  IIC_Wait_Ack();
    IIC_Stop();	
}

/*******************************************************************************
  * @函数名称	MS5611_init
  * @函数说明   初始化5611
  * @输入参数  	无
  * @输出参数   无
  * @返回参数   无
*******************************************************************************/
void MS5837_init(void)
{	 
  u8 inth,intl;
  u8 i; 
	 
	IIC_Init();	    
  delay_ms(20);	//初始化IIC PC11 PC12口子
	MS583703BA_RESET();	 // Reset Device  复位MS5837
	delay_ms(20);       //复位后延时（注意这个延时是一定必要的，可以缩短但似乎不能少于20ms）
	 
  for (i=0;i<7;i++) 
	{
		IIC_Start();
    IIC_Send_Byte(0xEC);
		IIC_Wait_Ack();
		IIC_Send_Byte(0xA0 + (i*2));
		IIC_Wait_Ack();
    IIC_Stop();
		delay_us(5);
		IIC_Start();
		IIC_Send_Byte(0xEC+0x01);  //进入接收模式
		delay_us(5);
		IIC_Wait_Ack();
		inth = IIC_Read_Byte(1);  		//带ACK的读数据
		delay_us(5);
		intl = IIC_Read_Byte(0); 			//最后一个字节NACK
		IIC_Stop();
    Cal_C[i] = (((uint16_t)inth << 8) | intl);
	}
	
	for(i=0;i<5;i++)
	{
		delay_ms(1);
		MS5837_Getdata();   //获取大气压
    Atmdsphere_Pressure+=Pressure;	
		printf("%d\t",Pressure);               //串口输出原始数据
	}
	Atmdsphere_Pressure=Atmdsphere_Pressure/5; //采集5次求平均值
	printf("Atmdsphere_Pressure:%d\r\n",Atmdsphere_Pressure);               //串口输出原始数据
}

/**************************实现函数********************************************
*函数原型:unsigned long MS561101BA_getConversion(void)
*功　　能:    读取 MS5837 的转换结果 
*******************************************************************************/
unsigned long MS583703BA_getConversion(uint8_t command)
{
	unsigned long conversion = 0;
	u8 temp[3];

	IIC_Start();
	IIC_Send_Byte(0xEC); 		//写地址
	IIC_Wait_Ack();
	IIC_Send_Byte(command); //写转换命令
	IIC_Wait_Ack();
	IIC_Stop();

	delay_ms(10); //ADC转换的延时
	IIC_Start();
	IIC_Send_Byte(0xEC); 		//写地址
	IIC_Wait_Ack();
	IIC_Send_Byte(0);				// start read sequence
	IIC_Wait_Ack();
	IIC_Stop();
	delay_ms(1);

	IIC_Start();
	IIC_Send_Byte(0xEC+0x01);  //进入接收模式
	IIC_Wait_Ack();
	temp[0] = IIC_Read_Byte(1);  //带ACK的读数据  bit 23-16
	temp[1] = IIC_Read_Byte(1);  //带ACK的读数据  bit 8-15
	temp[2] = IIC_Read_Byte(0);  //带NACK的读数据 bit 0-7
	IIC_Stop();

	conversion = (unsigned long)temp[0] * 65536 + (unsigned long)temp[1] * 256 + (unsigned long)temp[2];
	return conversion;
}

///***********************************************
//  * @brief  读取气压
//  * @param  None
//  * @retval None
//************************************************/
void MS5837_Getdata(void)
{
	D1_Pres= MS583703BA_getConversion(0x48); //4096分辨率
	delay_ms(5);
	D2_Temp = MS583703BA_getConversion(0x58);
	
	if(D2_Temp > (((uint32_t)Cal_C[5])*256l))
	{		
		dT=D2_Temp - (((uint32_t)Cal_C[5])*256l); //温度差
		TEMP=2000+dT*((uint32_t)Cal_C[6])/8388608LL;	
		OFF_=(uint32_t)Cal_C[2]*65536l+((uint32_t)Cal_C[4]*dT)/128l;
		SENS=(uint32_t)Cal_C[1]*32768l+((uint32_t)Cal_C[3]*dT)/256l;		//Pressure= ((D1_Pres*SENS/2097152l-OFF_)/8192l)/10;
	}
	else 
	{	
		dT=(((uint32_t)Cal_C[5])*256l) - D2_Temp;			
		TEMP=2000l-dT*((uint32_t)Cal_C[6])/8388608LL;	
		OFF_=(uint32_t)Cal_C[2]*65536l-((uint32_t)Cal_C[4]*dT)/128l;
		SENS=(uint32_t)Cal_C[1]*32768l-((uint32_t)Cal_C[3]*dT)/256l;		
	}
//	printf("1:%d\t%d\t%llu\t%llu\t%f\t%llu\r\n",D1_Pres,D2_Temp,dT,TEMP,OFF_,SENS);
//printf("%d\t",Cal_C[5]);
	
	//second order offset
	if(TEMP<2000)  // low temp
	{
	  Aux = (2000-TEMP)*(2000-TEMP);	
		T2 = 3*(dT*dT)/8589934592LL; 
		OFF2 = 3*Aux/2;
		SENS2 = 5*Aux/8;	
//		printf("2:%f\t%d\t%d\t%d\r\n",Aux,T2,OFF2,SENS2);		
	}
	else
	{
//		printf("!!");
		
	  Aux = (TEMP-2000)*(TEMP-2000);		
	  T2 = 2*(dT*dT)/137438953472LL;
		OFF2 = 1*Aux/16;
		SENS2 = 0;	
	}
	C5=((uint32_t)Cal_C[6]);
	OFF_ = OFF_ - OFF2;
	SENS = SENS - SENS2;	//二阶SENS和OFF_
	Temperature=(TEMP-T2)/100.0;
	Pressure= ((D1_Pres*SENS/2097152l-OFF_)/8192l)/10;
	P0= ((D1_Pres*SENS/2097152l-OFF_)/8192l)-Pressure*10;//用余数+整数的方法
	Depth=0.0102*(Pressure+(P0/10.0)-Atmdsphere_Pressure);	 //这个算数方法不太好，而且精度不够高
}
