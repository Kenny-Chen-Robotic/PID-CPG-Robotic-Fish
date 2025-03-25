#ifndef __MS5837_H_
#define __MS5837_H_
#include "sys.h"

void MS5837_init(void);
void MS5837_Getdata(void);

extern signed int dT,TEMP;
extern uint32_t Pressure,Temperature;
extern double Depth,P0;
extern uint32_t C5;	
extern uint32_t Atmdsphere_Pressure;
extern uint32_t Cal_C[7];

#endif

