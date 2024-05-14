#include "main.hpp"

#include "gpio.hpp"
#include "usart.hpp"
#include "delay.hpp"
#include "pid.hpp"

#include <string>
#include <vector>
#include "math.h"
#include <cstdlib>
#include <iostream>

#include "bias2.hpp"
extern double temp_2[500];
extern uint16_t bias_2[500];



int res = 0;




#define UART_LOG_XY
#define UART_LOG_XY_GRAPH
//#define UART_LOG_XY_TABLE

uint16_t spi_rx16[14];
volatile uint8_t rx1[18];
volatile uint8_t rx1_s[18];
uint8_t rx2[10] __attribute__((used));
uint8_t ptr=0;
uint8_t ptr2=0;
uint16_t baddr=700;
int16_t bias_off=-173+26+77-58+16+64;
uint16_t threshold = 0x12d;

volatile uint8_t cntspi2=0;
volatile uint8_t cnt_v[2];
volatile uint8_t spi2_v[5];
volatile uint8_t lev;
volatile uint8_t pkuc[6];
volatile uint8_t gcnt;
int val_uart;
//int32_t StartPageAddr = 0x08008000;
//uint64_t Data[6]= {0x31111, 0x42222, 0x3333, 0x4444, 0x5555, 0x6666};
uint8_t *TxBuffer;
uint8_t RxBuffer[100];
uint8_t TxCounter = 0, RxCounter = 0;

//std::vector<char> TxData; // Вектор целых чисел
uint16_t ctr=0;//,dt=0;
unsigned short DATA_ADC[10];
double vol_arr_temp[10],d_temp=0;;
double vol_arr[10];

unsigned int DATA_ADCacc;
volatile unsigned int DATA_ADCaccout;
unsigned int DATA_ADCacc1;
volatile unsigned int DATA_ADCaccout1;
unsigned int DATA_ADCacc2;
volatile unsigned int DATA_ADCaccout2;
unsigned int DATA_ADCacc3;
volatile unsigned int DATA_ADCaccout3;
volatile unsigned int DATA_ADCaccoutx;
extern volatile unsigned short temp[2001];

uint8_t rx0[9];
char buffer2[500];
char buffer[100];
double temp_work=-20;
volatile double temp_int;
 double pwm,PID,temp_current,temp_delta,temp_i,temp_d,
kp=100,
ki=0.25,
kd=5,
P,I,D;

 double x__,y__;




uint32_t val,reg_max=200,reg_min=0,dt=0;
uint8_t pin_ready=9,
        led_red=11,
        led_green=10,
        spi_sck=10,
        spi_miso=11,
        spi_mosi=12,
        spi_nss=15,

        pin_dac_1=4,
        pin_dac_2=5,
        pin_adc_0=0,
        pin_adc_1=1,
        pin_adc_2=2,
        pin_adc_3=3,
        pin_adc_4=4,
        pin_adc_5=5,
        pin_adc_6=6,
        pin_adc_7=7,
        pin_adc_8=0,
        pin_adc_9=1,
        E9=13,
        DONE=8,
        PROG_B=2;


gpio gpio_stm32f405;
usart uart_1;
pid pid_int;



double volt=0;
double temp_ext=0,temp_rad=0,V_bias=0;

double _TransferFunction(double voltage)
  {
      return(-1481.96 + sqrt((2.1962 * pow(10, 6)) + (1.8639 - voltage) / (3.88 * pow(10, -6))));
  }

void delay_ms(uint32_t us)
{
    RCC->APB2ENR |=RCC_APB2ENR_TIM8EN;
    TIM8->PSC = 0; // Настройка предделителя
    TIM8->ARR = 8000-1; // Настройка автоперезагрузки (при 80 MHz тактовой частоте это будет 1 мкс) 
    TIM8->CNT = 0; // Сброс счетчика
    TIM8->CR1 = TIM_CR1_CEN; // Включение таймера

    while(us--)
    {
        while(!(TIM8->SR & TIM_SR_UIF)); // Ждем, пока флаг обновления не станет 1
        TIM8->SR &= ~TIM_SR_UIF; // Сброс флага обновления
    }
    TIM8->CR1 = 0; // Выключение таймера
}

// ������� ��� ���������� �������� ����������������� ������� � ����� x
double lagrangeInterpolation(double x, const std::vector<double>& x_points, const std::vector<double>& y_points) {
    double result = 0.0;

    for (size_t i = 0; i < x_points.size(); ++i) {
        double term = y_points[i];
        for (size_t j = 0; j < x_points.size(); ++j) {
            if (i != j) {
                term *= (x - x_points[j]) / (x_points[i] - x_points[j]);
            }
        }
        result += term;
    }

    return result;
}

void delay_us(uint32_t us)
{
    RCC->APB2ENR |=RCC_APB2ENR_TIM8EN;
    TIM8->PSC = 0; // Настройка предделителя
    TIM8->ARR = 80-1; // Настройка автоперезагрузки (при 80 MHz тактовой частоте это будет 1 мкс) 
    TIM8->CNT = 0; // Сброс счетчика
    TIM8->CR1 = TIM_CR1_CEN; // Включение таймера

    while(us--)
    {
        while(!(TIM8->SR & TIM_SR_UIF)); // Ждем, пока флаг обновления не станет 1
        TIM8->SR &= ~TIM_SR_UIF; // Сброс флага обновления
    }
    TIM8->CR1 = 0; // Выключение таймера
}

void ADC_SCAN (void)
{
  int val_avg=1000;
  for(int z=0;z<10;z++)
  {
   vol_arr_temp[z]=0;
  }

   for(int i=0;i<val_avg;i++)
        {
    for(int j=0;j<10;j++)
        {
          ADC1->SQR3 = j;
          ADC1->CR2 |= ADC_CR2_SWSTART;

          while(!(ADC1->SR & ADC_SR_EOC)){}
          vol_arr_temp[j]+=ADC1->DR*(3.3/4096);
          vol_arr_temp[j]=std::round(vol_arr_temp[j]*100)/100;
        }
        }
        for(int i=0;i<10;i++)
        {
          vol_arr[i]=vol_arr_temp[i]/val_avg;
          vol_arr[i]=std::round(vol_arr[i]*100000)/100000;
        }
      // sprintf(buffer2, ">ADC_temp:%d\n",vol_arr_temp[2]);
      // uart_1.uart_tx_data(buffer2);
  V_bias=((vol_arr[0])*32.8);//
  temp_int=_TransferFunction(vol_arr[2]);
  temp_ext=_TransferFunction(vol_arr[6]*2);
  temp_rad=_TransferFunction(vol_arr[9]*100);


sprintf(buffer2, ">temp_int:%-8.1f\n",temp_int);
uart_1.uart_tx_data(buffer2);
// sprintf(buffer2, ">temp_ext:%-8.2f\n",temp_ext);
// uart_1.uart_tx_data(buffer2);
// sprintf(buffer2, ">temp_rad:%-8.2f\n",temp_rad);
// uart_1.uart_tx_data(buffer2);
// sprintf(buffer2, ">v_bias:%-8.2f\n",V_bias);
// uart_1.uart_tx_data(buffer2);
	if (temp_int>=temp_work+1)
	{
	LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_11); //red
	LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_9); //ready
	LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_10); //green
	}
	else
	{
	LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_11); //red
	LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_9); //ready
	LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_10); //green
	}
}
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_Init1msTick(16000000);
  LL_SetSystemCoreClock(16000000);
}
// void SystemClock_Config(void)
// {
//   // Установка задержки Flash
//   FLASH->ACR &= ~FLASH_ACR_LATENCY;
//   while ((FLASH->ACR & FLASH_ACR_LATENCY) != 0)
//   {
//   }

//   // Включение HSI
//   RCC->CR |= RCC_CR_HSION;
//   while ((RCC->CR & RCC_CR_HSIRDY) == 0)
//   {
//   }

//   // Установка предделителей шин
//   RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);

//   // Установка источника системного тактового сигнала
//   RCC->CFGR &= ~RCC_CFGR_SW;
//   RCC->CFGR |= RCC_CFGR_SW_HSI;
//   while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
//   {
//   }

//   // Инициализация SysTick
//   SysTick->LOAD = 16000000 - 1;
//   SysTick->VAL = 0;
//   SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

//   // Установка SystemCoreClock
//   SystemCoreClock = 16000000;
// }
void MX_ADC1_Init(void)
{
  /* Peripheral clock enable */
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  //RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
 // RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

  /* GPIO Configuration */
 // GPIOA->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER2 | GPIO_MODER_MODER3 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7;
 //GPIOB->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1;

  /* ADC Configuration */
 //ADC1->CR1 |= 0x1<<ADC_CR1_RES_Pos;
// 00: 12-bit (15 ADCCLK cycles)
// 01: 10-bit (13 ADCCLK cycles)
// 10: 8-bit (11 ADCCLK cycles)
// 11: 6-bit (9 ADCCLK cycles)
  ADC1->CR2 &= ~ADC_CR2_ALIGN;
  ADC1->CR1 &= ~ADC_CR1_SCAN;
  ADC1->CR2 &= ~ADC_CR2_CONT;
  ADC1->CR2 &= ~ADC_CR2_DMA;
  ADC1->CR2 |= ADC_CR2_EOCS;
  ADC->CCR &= ~ADC_CCR_MULTI;
  ADC->CCR |= ADC_CCR_ADCPRE;

  /* ADC Channel Configuration */
  // ADC1->SQR3 &= ~ADC_SQR3_SQ1;
  //ADC1->SMPR2 |=0x7FFFFFF;
   ADC1->SMPR2 |=0x0;
  // ADC1->SMPR2 &= ~ADC_SMPR2_SMP0;
  // ADC1->SMPR2 |= ADC_SMPR2_SMP0_0 | ADC_SMPR2_SMP0_1;

  /* ADC Enable */
  ADC1->CR2 |= ADC_CR2_ADON;
}

void MX_DAC_Init(void)
{
  /* Peripheral clock enable */
  RCC->APB1ENR |= RCC_APB1ENR_DACEN;
  //RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

  /* GPIO Configuration */
  //GPIOA->MODER |= GPIO_MODER_MODER4 | GPIO_MODER_MODER5;

  /* DAC Configuration */
  DAC->CR &= ~DAC_CR_TSEL1;
  DAC->CR &= ~DAC_CR_WAVE1;
  //DAC->CR |= DAC_CR_BOFF1;
  DAC->CR &= ~DAC_CR_BOFF1;

  /* DAC Channel 2 Configuration */
  DAC->CR &= ~DAC_CR_TSEL2;
  DAC->CR &= ~DAC_CR_WAVE2;
  // DAC->CR |= DAC_CR_BOFF2;
  DAC->CR &=~ DAC_CR_BOFF2;

  /* DAC Enable */
  DAC->CR |= DAC_CR_EN1 | DAC_CR_EN2;
}

void MX_SPI2_Init(void)
{
  /* Peripheral clock enable */
  RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

  /* GPIO Configuration */
  GPIOB->MODER |= GPIO_MODER_MODER12_1 | GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1;
  GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR12 | GPIO_OSPEEDER_OSPEEDR13 | GPIO_OSPEEDER_OSPEEDR14 | GPIO_OSPEEDER_OSPEEDR15;
  GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_12 | GPIO_OTYPER_OT_13 | GPIO_OTYPER_OT_14 | GPIO_OTYPER_OT_15);
  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR12 | GPIO_PUPDR_PUPDR13 | GPIO_PUPDR_PUPDR14 | GPIO_PUPDR_PUPDR15);
  GPIOB->AFR[1] |= 0x55550000;

  /* SPI Configuration */
  SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM | SPI_CR1_BR_0;
 // SPI2->CR2 = SPI_CR2_FRXTH | SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2; ????
  SPI2->CRCPR = 10;

  /* SPI Enable */
  SPI2->CR1 |= SPI_CR1_SPE;
}

static void MX_SPI3_Init(void)
{
  LL_SPI_InitTypeDef SPI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  /**SPI3 GPIO Configuration
  PA15   ------> SPI3_NSS
  PC10   ------> SPI3_SCK
  PC11   ------> SPI3_MISO
  PC12   ------> SPI3_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = 1;//LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10|LL_GPIO_PIN_11|LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;//LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_HARD_OUTPUT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct.BitOrder = LL_SPI_LSB_FIRST;//LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI3, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI3, LL_SPI_PROTOCOL_MOTOROLA);
}

void MX_TIM14_Init(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
  NVIC_SetPriority(TIM8_TRG_COM_TIM14_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);
  TIM14->PSC = 0x0008;
  TIM14->ARR = 0x1000;
  TIM14->CR1 |= TIM_CR1_ARPE;
  TIM14->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_0;
  TIM14->CCER &= ~TIM_CCER_CC1E;
  TIM14->CCR1 = 0;
  TIM14->CCER &= ~TIM_CCER_CC1P;
  TIM14->BDTR |= TIM_BDTR_MOE;
  TIM14->DIER |= TIM_DIER_UIE;
  TIM14->CR1 |= TIM_CR1_CEN;
}

void MX_TIM11_Init(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
  NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
  TIM11->PSC = 0x0008;
  TIM11->ARR = 0x200;
  TIM11->CR1 |= TIM_CR1_ARPE;
  TIM11->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_0;
  TIM11->CCER &= ~TIM_CCER_CC1E;
  TIM11->CCR1 = 0;
  TIM11->CCER &= ~TIM_CCER_CC1P;
  TIM11->BDTR |= TIM_BDTR_MOE;
  TIM11->DIER |= TIM_DIER_UIE;
  TIM11->CR1 |= TIM_CR1_CEN;
}

void MX_TIM3_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Enable TIM3 clock
    TIM3->PSC = 8-1; // Set prescaler to 16 (SystemCoreClock = 16MHz)
    TIM3->ARR = 1000; // Set auto-reload to 10
    TIM3->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; // Set output compare 3 mode to PWM1
    TIM3->CCER |= TIM_CCER_CC3E; // Enable the output for channel 3
    TIM3->CCR3 =0;// 44000; // Set the duty cycle to 50%
    TIM3->CR1 |= TIM_CR1_CEN; // Start TIM3
}

volatile double count_1,count_2,x[1500],y[1500];

void resolve(double x1,double x2,double x3,double y1,double y2,double y3,double accuracy)
{
  for ( int i = -50; i<count_1; i++)
   {
    count_1=(-(x2-x1)/accuracy);//0.1
    x[i]=x1-i*accuracy;
    y[i]=y2+((y1-y2)/(x1-x2))*(x[i]-x2);
     #ifdef UART_LOG_XY_GRAPH
    sprintf(buffer,">Bias=f(temp):%f:%f|xy\n", x[i],y[i]);
    uart_1.uart_tx_data(buffer);
   #endif
  #ifdef UART_LOG_XY_TABLE
   sprintf(buffer,"{%-8.1f,%d},\n", x[i],(int)y[i]);
   // sprintf(buffer,"x=%f,y=:%f|xy\n", x[i],y[i]);
    uart_1.uart_tx_data(buffer);
   // delay_ms(200);
   #endif
   }

   for (int  i =0; i<count_2+200; i++)
   {
    count_2=(-(x3-x2)/accuracy);
    x[i]=x2-i*accuracy;
    y[i]=y3+((y2-y3)/(x2-x3))*(x[i]-x3);
     #ifdef UART_LOG_XY_GRAPH
    sprintf(buffer,">Bias=f(temp):%f:%f|xy\n", x[i],y[i]);
    uart_1.uart_tx_data(buffer);
   #endif
     #ifdef UART_LOG_XY_TABLE
  sprintf(buffer,"{%-8.1f,%d},\n", x[i],(int)y[i]);
    uart_1.uart_tx_data(buffer);
    
   #endif
   }
// delay_ms(200);
}

int main()
{
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  SystemClock_Config();

//gpio gpio_stm32f405;
//UASRT1_gpio PA9 -> Tx  PA10 -> Rx
gpio_stm32f405.gpio_init(GPIOA,9,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_alternate,gpio_speed::gpio_speed_very_high,gpio_pull_up_down::pull_up);
gpio_stm32f405.gpio_init(GPIOA,10,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_alternate,gpio_speed::gpio_speed_very_high,gpio_pull_up_down::pull_up);
gpio_stm32f405.config_af(GPIOA,9,7);//AF7
gpio_stm32f405.config_af(GPIOA,10,7);//AF7
//GPIO input/output
gpio_stm32f405.gpio_init(GPIOB,pin_ready,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_output,gpio_speed::gpio_speed_very_high,gpio_pull_up_down::no_pull_up_down);
gpio_stm32f405.gpio_init(GPIOB,led_red,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_output,gpio_speed::gpio_speed_very_high,gpio_pull_up_down::no_pull_up_down);
gpio_stm32f405.gpio_init(GPIOB,led_green,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_output,gpio_speed::gpio_speed_very_high,gpio_pull_up_down::no_pull_up_down);
//SPI_3
gpio_stm32f405.gpio_init(GPIOC,spi_sck,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_alternate,gpio_speed::gpio_speed_very_high,gpio_pull_up_down::no_pull_up_down);
gpio_stm32f405.gpio_init(GPIOC,spi_mosi,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_alternate,gpio_speed::gpio_speed_very_high,gpio_pull_up_down::no_pull_up_down);
gpio_stm32f405.gpio_init(GPIOC,spi_miso,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_alternate,gpio_speed::gpio_speed_very_high,gpio_pull_up_down::no_pull_up_down);
gpio_stm32f405.gpio_init(GPIOA,spi_nss,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_output,gpio_speed::gpio_speed_very_high,gpio_pull_up_down::no_pull_up_down);
//DAC
gpio_stm32f405.gpio_init(GPIOA,pin_dac_1,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_analog,gpio_speed::gpio_speed_very_high,gpio_pull_up_down::no_pull_up_down);
gpio_stm32f405.gpio_init(GPIOA,pin_dac_2,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_analog,gpio_speed::gpio_speed_very_high,gpio_pull_up_down::no_pull_up_down);
//ADC
gpio_stm32f405.gpio_init(GPIOA,pin_adc_0,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_analog,gpio_speed::gpio_speed_very_high,gpio_pull_up_down::no_pull_up_down);
gpio_stm32f405.gpio_init(GPIOA,pin_adc_1,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_analog,gpio_speed::gpio_speed_very_high,gpio_pull_up_down::no_pull_up_down);
gpio_stm32f405.gpio_init(GPIOA,pin_adc_2,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_analog,gpio_speed::gpio_speed_very_high,gpio_pull_up_down::no_pull_up_down);
gpio_stm32f405.gpio_init(GPIOA,pin_adc_3,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_analog,gpio_speed::gpio_speed_very_high,gpio_pull_up_down::no_pull_up_down);
gpio_stm32f405.gpio_init(GPIOA,pin_adc_4,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_analog,gpio_speed::gpio_speed_very_high,gpio_pull_up_down::no_pull_up_down);
gpio_stm32f405.gpio_init(GPIOA,pin_adc_5,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_analog,gpio_speed::gpio_speed_very_high,gpio_pull_up_down::no_pull_up_down);
gpio_stm32f405.gpio_init(GPIOA,pin_adc_6,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_analog,gpio_speed::gpio_speed_very_high,gpio_pull_up_down::no_pull_up_down);
gpio_stm32f405.gpio_init(GPIOA,pin_adc_7,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_analog,gpio_speed::gpio_speed_very_high,gpio_pull_up_down::no_pull_up_down);
gpio_stm32f405.gpio_init(GPIOB,pin_adc_8,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_analog,gpio_speed::gpio_speed_very_high,gpio_pull_up_down::no_pull_up_down);
gpio_stm32f405.gpio_init(GPIOB,pin_adc_9,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_analog,gpio_speed::gpio_speed_very_high,gpio_pull_up_down::no_pull_up_down);
//gpio_stm32f405.gpio_init(GPIOB,pin_adc,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_analog,gpio_speed::gpio_speed_very_high,gpio_pull_up_down::no_pull_up_down);
//PWM
//gpio_stm32f405.gpio_init(GPIOA,2,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_alternate,gpio_speed::gpio_speed_very_high,gpio_pull_up_down::no_pull_up_down);
//gpio_stm32f405.config_af(GPIOA,2,1);//AF0
gpio_stm32f405.gpio_init(GPIOB,0,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_alternate,gpio_speed::gpio_speed_very_high,gpio_pull_up_down::no_pull_up_down);
gpio_stm32f405.config_af(GPIOB,0,2);//AF1

gpio_stm32f405.gpio_init(GPIOD,led_green,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_output,gpio_speed::gpio_speed_very_high,gpio_pull_up_down::no_pull_up_down);
//PLIS
gpio_stm32f405.gpio_init(GPIOA,DONE,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_input,gpio_speed::gpio_speed_very_high,gpio_pull_up_down::pull_down);
gpio_stm32f405.gpio_init(GPIOD,PROG_B,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_output,gpio_speed::gpio_speed_very_high,gpio_pull_up_down::no_pull_up_down);

  MX_ADC1_Init();
  MX_DAC_Init();
  MX_SPI3_Init();




LL_GPIO_SetOutputPin(GPIOA,LL_GPIO_PIN_11); //M0 SET

	rx1[4]=32;
	rx0[0]=170;
	rx0[1]=60;
	rx0[2]=0;//7 6 6
	rx0[3]=0xe0;//32; //55 159 213 245 240
	rx0[4]=7;//10;//2
	rx0[5]=64;//104;
	rx0[6]=1;//2;//4   2 1 2 mag- 3/190 gib- 2/137
	rx0[7]=52;//170;  //158 145 169 144 148 164 132 168 138
	rx0[8]=128;	//128


//LL_DAC_ConvertData12RightAligned (DAC1, LL_DAC_CHANNEL_1, rx0[7]+256*rx0[6]);
//LL_DAC_ConvertData12RightAligned (DAC1, LL_DAC_CHANNEL_2, rx0[5]+256*rx0[4]);

   //DAC->DHR12R1 = rx0[7] + 256 * rx0[6];
   //DAC->DHR12R2 = rx0[5] + 256 * rx0[4];


  while(gpio_stm32f405.get_state_pin(GPIOA,DONE)==0) //PLIS is run?
  {
  gpio_stm32f405.set_pin_state(GPIOD,PROG_B,0); //run plis.
  delay_ms(5000);
	gpio_stm32f405.set_pin_state(GPIOD,PROG_B,1);
  delay_ms(5000);
  }

  
  delay_ms(1000);
  uart_1.usart_init();
   // MX_SPI2_Init();
      MX_TIM11_Init();
  MX_TIM14_Init();
  MX_TIM3_Init();

SystemCoreClockUpdate();


	//	LL_SPI_Enable(SPI2);
std::vector<double> y_points{1423, 1493, 1545};
std::vector<double> x_points{-47, -35, -25};

//Custom code
rx1[3]=0xE0;
LL_DAC_ConvertData12RightAligned (DAC1, LL_DAC_CHANNEL_2, 0x100);//����������
 LL_DAC_ConvertData12RightAligned (DAC1, LL_DAC_CHANNEL_1, 0x12e);//�����
rx1[8]=0x00;
//Custom code

//resolve(-25,-35,-47,1535,1485,1423,0.1);



  while (1)
  {


   DWT_CYCCNT  = 0;
   DWT_CONTROL|= DWT_CTRL_CYCCNTENA_Msk;
   ADC_SCAN();
   //pid_int.start(temp_work);
    //TEMPER

//     if(RxBuffer[0]==0x54 && RxBuffer[4]==42)
//     {
// int s = 1;
// int i = -1;
// res = 0;

// if (RxBuffer[1] == '-') {
//   s = -1;
//   i = 1;
// }
// while (RxBuffer[++i] != 42) { //iterate until the array end
//   res = res*10 + (RxBuffer[i] - '0'); //generating the integer according to read parsed numbers.
// }
// res = res*s; //answer: -908
//    temp_work=res;
//    RxCounter=0;
//    for(int i=0;i<20;i++)
//    {
//     RxBuffer[i]=0;
//    }
//     }

//     //BIAS
//     if(RxBuffer[0]==0x54 && RxBuffer[1]==0x52 && RxBuffer[6]==42||RxBuffer[5]==42)
//     {

// int i = 1;
// res = 0;



// while (RxBuffer[++i] != 42) { //iterate until the array end
//   res = res*10 + (RxBuffer[i] - '0'); //generating the integer according to read parsed numbers.
// }
// if(res>100 && res<1792)
//   LL_DAC_ConvertData12RightAligned (DAC1, LL_DAC_CHANNEL_2, res);

//   RxCounter=0;
//      for(int i=0;i<20;i++)
//    {
//     RxBuffer[i]=0;
//    }
//     }

//      //BIAS
//     if(RxBuffer[0]==0x55 && RxBuffer[4]==42)
//     {
// //int s = 1;
// int i = 0;


// // if (RxBuffer[0] ==0x55)
// //  i = 1;

// while (RxBuffer[++i] != 42) { //iterate until the array end
//   res = res*10 + (RxBuffer[i] - '0'); //generating the integer according to read parsed numbers.
// }
// if(res>287 && res<336)
//   LL_DAC_ConvertData12RightAligned (DAC1, LL_DAC_CHANNEL_1, res);
//  res=0;
//   RxCounter=0;
//      for(int i=0;i<20;i++)
//    {
//     RxBuffer[i]=0;
//    }
//     }


 //pid_int.calc_PID(temp_work,27000,0,0);

		rx1_s[0]=rx1[0];
		if ((temp_int<=temp_work+1))
		{
    rx1_s[1]=temp[1000]>>8;//DATA_ADCaccout2>>8;
		rx1_s[2]=temp[1000]&255;//DATA_ADCaccout2&255;
		// rx1_s[1]=temp[5000/2-1500]>>8;//DATA_ADCaccout2>>8;
		// rx1_s[2]=temp[5000/2-1500]&255;//DATA_ADCaccout2&255;
		}
		else
		{
			if (temp_int<-70)
			{
				rx1_s[1]=2;
				rx1_s[2]=0;
			}
			else
			{
				rx1_s[1]=0;
				rx1_s[2]=100;
			}
    }

DATA_ADCaccout1=747;
DATA_ADCaccout3=4419;
		rx1_s[3]=rx1[3];
		rx1_s[4]=DATA_ADCaccout1>>8;//
		rx1_s[5]=DATA_ADCaccout1&255;
		rx1_s[6]=1;
		rx1_s[7]=3;
		rx1_s[8]=temp[DATA_ADCaccout3/2-1500]>>8;//DATA_ADCaccout3>>8;//
		rx1_s[9]=temp[DATA_ADCaccout3/2-1500]&255;//DATA_ADCaccout3&255;
		rx1_s[10]=1;
		rx1_s[11]=10;
		rx1_s[12]=1;
		rx1_s[13]=0;

     /*sprintf(buffer2, ">spi_rx16[0]:%d\n",spi_rx16[0]);
      uart_1.uart_tx_data(buffer2);
      sprintf(buffer2, ">spi_rx16[1]:%d\n",spi_rx16[1]);
      uart_1.uart_tx_data(buffer2);
      sprintf(buffer2, ">spi_rx16[2]:%d\n",spi_rx16[2]);
      uart_1.uart_tx_data(buffer2);
      sprintf(buffer2, ">spi_rx16[3]:%d\n",spi_rx16[3]);
      uart_1.uart_tx_data(buffer2);*/
      sprintf(buffer2, ">count:%d\n", static_cast<int>(((static_cast<uint32_t>(spi_rx16[1])<<16) + static_cast<uint32_t>(spi_rx16[0]))/2));
      uart_1.uart_tx_data(buffer2);
        sprintf(buffer2, ">TR:%d\n", static_cast<int>(DAC1->DHR12R1));
       uart_1.uart_tx_data(buffer2);
         sprintf(buffer2, ">U:%d\n", static_cast<int>(DAC1->DHR12R2));
       uart_1.uart_tx_data(buffer2);

       

   // �����, ��� ������� ����� ��������� �������� �������
//     for(int i=0;i<500;i++)
//     {
//  // ���������� �������� ����������������� ������� � ����� x
//    x__ = temp_2[i];
//     y__ = lagrangeInterpolation(x__, x_points, y_points);
//        sprintf(buffer2, ">V_bias_new:%f\n",y__);
//         uart_1.uart_tx_data(buffer2);
//     }
//    temp_2[0]=0;

   


  }


}

      extern "C" void USART1_IRQHandler(void)
      {
    if (USART1->SR & USART_SR_RXNE) // Проверка флага RXNE
    {
        RxBuffer[RxCounter++] = USART1->DR; // Прием данных
         //    uart_1.uart_tx_byte(USART1->DR);
         //    uart_1.uart_tx_byte(10);//\n
             //char value_T='T';
            //  if(USART1->DR==value_T)
            //  {

            //  }
    }
}

extern "C" void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
   TIM11->SR=0;//reset flag


   //pid_int.calc_PID(temp_work,27000,0,0);
}


extern "C" void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
  int i_temp=0;
	TIM14->SR=0;
	ptr=0;
	gcnt++;

	if (rx1[8]<100)
	{
	if (temp_int<=temp_work+1)
		{
			// //baddr=DATA_ADCaccout2-4899;
			// LL_DAC_ConvertData12RightAligned (DAC1, LL_DAC_CHANNEL_2, (bias[baddr]+rx1[6]+bias_off));
			// LL_DAC_ConvertData12RightAligned (DAC1, LL_DAC_CHANNEL_1, threshold);
		temp_int=round(temp_int*10)/10;
     for(int i=0;i<500;i++)
     {
        if(temp_int==temp_2[i])
        {
          i_temp=i;
           LL_DAC_ConvertData12RightAligned (DAC1, LL_DAC_CHANNEL_2,bias_2[i_temp]);
          break;
        }
     }
 

    	//LL_DAC_ConvertData12RightAligned (DAC1, LL_DAC_CHANNEL_2, bias2[][]);
    
    }
	else
	{
		if (temp_int<=temp_work+1)
		{
			 LL_DAC_ConvertData12RightAligned (DAC1, LL_DAC_CHANNEL_2, 0x100);
		}
	}
}

	LL_GPIO_ResetOutputPin(GPIOA,LL_GPIO_PIN_15);
	LL_SPI_Enable(SPI3);
	spi_rx16[0]=0;
			spi_rx16[1]=0;
			spi_rx16[2]=0;
			spi_rx16[3]=0;
			while (ptr<14)
			{
			LL_SPI_TransmitData16(SPI3,rx1_s[ptr]+256*rx1_s[ptr+1]); //0x0060
			 while (LL_SPI_IsActiveFlag_TXE(SPI3)==0) //LL_SPI_IsActiveFlag_BSY(SPI3)
				{
				}
			 while (LL_SPI_IsActiveFlag_RXNE(SPI3)==0) //LL_SPI_IsActiveFlag_BSY(SPI3)
				{
				}
			 spi_rx16[ptr>>1]=LL_SPI_ReceiveData16(SPI3);
			 ptr=ptr+2;
			}
			LL_GPIO_SetOutputPin(GPIOA,LL_GPIO_PIN_15);
			LL_SPI_Disable(SPI3);
			cnt_v[1]=spi_rx16[1];//cnt_v[1]=spi_rx16[1];
			cnt_v[0]=spi_rx16[1]>>8;
			pkuc[0]=spi_rx16[2]>>12;
			pkuc[1]=spi_rx16[2]&0x00ff;
			pkuc[2]=(spi_rx16[2]>>8)&0x0f;
			pkuc[3]=spi_rx16[3]&0x00ff;
			pkuc[4]=(spi_rx16[3]>>8)&0x0f;

 		return;
	}


void Error_Handler(void)
{

  __disable_irq();
  while (1)
  {
  }

}
