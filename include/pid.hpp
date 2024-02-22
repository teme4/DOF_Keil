
#ifndef _PID_
#define _PID_
#include "stm32f4xx.h"
#include "delay.hpp"
#include "stdio.h"
#include "usart.hpp"

double PID,temp_current,temp_delta,temp_last,
kp=27000,
ki=0,
kd=0,
P,I,D;

extern double temp_int;

extern usart uart_1;

uint32_t val,pwm,reg_max=27000,reg_min,time=0;
char buffer[100];

class pid
{
private:

public:
 void start(double set_temp);
};

 #endif //PID