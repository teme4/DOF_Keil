#include "pid.hpp"

 double integral=0,prevTemp=0;
void pid::start(double set_temp)
{
    sprintf(buffer, ">set:%f\n",set_temp);
    uart_1.uart_tx_data(buffer);
    temp_delta=temp_int-set_temp;//-
    kp=150,//50
    ki=0.001,//0.001
    integral+=temp_delta*ki;
    pwm=temp_delta*kp+integral;//+D*kd;

    if(pwm<0)
    pwm*=-1;

    sprintf(buffer, ">P:%f\n",temp_delta*kp);
    uart_1.uart_tx_data(buffer);
    sprintf(buffer, ">I:%f\n",integral);
    uart_1.uart_tx_data(buffer);

    if(pwm>TIM3->ARR)//>
    {
        pwm =TIM3->ARR;//-pwm;
         //pwm
    }
    if(pwm<reg_min)//<
        pwm =reg_min;//-pwm;
    TIM3->CCR3 =(uint16_t)pwm;
    sprintf(buffer, ">PWM:%d\n",(uint16_t)pwm);
    uart_1.uart_tx_data(buffer);

    sprintf(buffer, ">temp_delta:%-8.1f\n",temp_delta);
    uart_1.uart_tx_data(buffer);
   }

