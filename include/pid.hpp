
#ifndef _PID_
#define _PID_
#include "stm32f4xx.h"
#include "delay.hpp"
#include "stdio.h"
#include "usart.hpp"
extern volatile double temp_int;
extern double pwm,PID,temp_current,temp_delta,temp_i,temp_d,
kp,
ki,
kd,
P,I,D;

extern uint32_t val,reg_max,reg_min,dt;
extern char buffer[100];
extern usart uart_1;


class pid
{
private:

public:
 void start(double set_temp);
 double calc_PID(double setpoint,double kp,double ki,double kd);
};

 #endif //PID