
#ifndef _PID_
#define _PID_
#include "stm32f4xx.h"
#include "delay.hpp"
#include "stdio.h"
#include "usart.hpp"

extern double PID,temp_current,temp_delta,temp_last,temp_int,
kp,
ki,
kd,
P,I,D;

extern uint32_t val,pwm,reg_max,reg_min,time;
extern char buffer[100];
extern usart uart_1;


class pid
{
private:

public:
 void start(double set_temp);
};

 #endif //PID