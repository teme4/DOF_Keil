#include "pid.hpp"





 void pid::start(double set_temp)
  {
temp_current=temp_int;
temp_delta=temp_current-set_temp;
DWT_CONTROL &= ~DWT_CTRL_CYCCNTENA_Msk;
time=DWT_CYCCNT/1000;
sprintf(buffer, ">time_of_one_cycle:%d\n",time);
 uart_1.uart_tx_data(buffer);
sprintf(buffer, ">temp_delta:%-8.2f\n",temp_delta);
 uart_1.uart_tx_data(buffer);
sprintf(buffer, ">set:%f\n",set_temp);
 uart_1.uart_tx_data(buffer);
  pwm=temp_delta*kp+temp_delta*time*0.1;

 if(pwm<reg_min)
  pwm =reg_min;

 if(pwm>reg_max)
  pwm =reg_max;

 TIM3->CCR3 =pwm;
 double _pwm=pwm/100;
sprintf(buffer, ">PWM:%d\n",pwm);
 uart_1.uart_tx_data(buffer);
  }

