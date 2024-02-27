#include "pid.hpp"


 void pid::start(double set_temp)
{
double integral=0,prevTemp=0;
temp_delta=temp_int-set_temp;
sprintf(buffer, ">temp_delta:%-8.2f\n",temp_delta);
uart_1.uart_tx_data(buffer);
DWT_CONTROL &= ~DWT_CTRL_CYCCNTENA_Msk;
dt=DWT_CYCCNT/1000;
integral+=temp_delta*dt*ki;
sprintf(buffer, ">PID_I:%-8.2f\n",integral*ki);
 uart_1.uart_tx_data(buffer);
sprintf(buffer, ">set:%f\n",set_temp);
 uart_1.uart_tx_data(buffer);

double D=(temp_delta-prevTemp)/dt;
sprintf(buffer, ">PID_D:%f\n",D);
uart_1.uart_tx_data(buffer);
prevTemp=temp_delta;

 pwm=temp_delta*kp+integral*ki+D*kd;
 if(pwm<reg_min)
  pwm =reg_min;

 if(pwm>reg_max)
  pwm =reg_max;

 TIM3->CCR3 =pwm;
 double _pwm=pwm/100;
sprintf(buffer, ">PWM:%d\n",pwm);
 uart_1.uart_tx_data(buffer);
  }

double pid::calc_PID(double setpoint,double kp,double ki,double kd)
{
double err=setpoint-temp_int;
double integral=0,prevErr=0;
integral+=err*dt;
sprintf(buffer, ">integral:%f\n",integral);
uart_1.uart_tx_data(buffer);
double D=(err-prevErr)/dt;
sprintf(buffer, ">D:%f\n",D);
uart_1.uart_tx_data(buffer);
prevErr=err;
 TIM3->CCR3 =err*kp+integral*ki+D*kd;
 sprintf(buffer, ">PID:%f\n", TIM3->CCR3);
 uart_1.uart_tx_data(buffer);
//return (err*kp+integral*ki+D*kd);
}
