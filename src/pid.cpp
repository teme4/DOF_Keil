#include "pid.hpp"


void pid::start(double set_temp)
{
    double integral=0,prevTemp=0;
    temp_delta=temp_int-set_temp;
    sprintf(buffer, ">temp_delta:%-8.2f\n",temp_delta);
    uart_1.uart_tx_data(buffer);
    DWT_CONTROL &= ~DWT_CTRL_CYCCNTENA_Msk;
    dt=DWT_CYCCNT/1000;
    sprintf(buffer, ">dt:%d\n",static_cast<int>(dt));
    uart_1.uart_tx_data(buffer);
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
    pwm+=100;

    if(pwm>reg_max) 
        pwm =reg_max;//-pwm;
    if(pwm<reg_min) 
        pwm =reg_min;//-pwm;
    //TIM3->CCR3 =(uint16_t)pwm;
    TIM3->CCR3 =500;
    sprintf(buffer, ">PWM:%-8.2f\n",pwm);
    uart_1.uart_tx_data(buffer);
 }

// double pid::calc_PID(double setpoint,double kp,double ki,double kd)
// {
// double pwm,integral=0,prevErr=0,err=setpoint-temp_int;
// integral+=err*dt;
// sprintf(buffer, ">integral:%f\n",integral);
// uart_1.uart_tx_data(buffer);
// double D=(err-prevErr)/dt;
// sprintf(buffer, ">D:%f\n",D);
// uart_1.uart_tx_data(buffer);
// prevErr=err;
// pwm=err*kp+integral*ki+D*kd;
// //  if(pwm>reg_max)
// //   pwm =reg_max;
//  TIM3->CCR3 =pwm;
//  sprintf(buffer, ">PWM:%f\n",pwm);
//  uart_1.uart_tx_data(buffer);
// return (err*kp+integral*ki+D*kd);
// }

// (вход, установка, п, и, д, период в секундах, мин.выход, макс. выход)
// int computePID(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut) {
//   float err = setpoint - input;
//   static float integral = 0, prevErr = 0;
//   integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);
//   float D = (err - prevErr) / dt;
//   prevErr = err;
//   return constrain(err * kp + integral + D * kd, minOut, maxOut);
// }