#include "main.hpp"
#include "gpio.hpp"
#include "usart.hpp"
#include <string>
#include <vector>
#include "math.h"


uint8_t *TxBuffer;
uint8_t RxBuffer[100];
uint8_t TxCounter = 0, RxCounter = 0;
std::vector<char> TxData; // Вектор целых чисел


uint16_t ctr=0,dt=0;
unsigned short DATA_ADC[10];
double vol_arr_temp[10];
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
extern uint8_t rx1[18];
extern uint8_t rx1_s[18];
extern volatile unsigned short temp[2001];
uint8_t rx0[9];

uint8_t pin_ready=9,
        led_red=11,
        led_green=10,
        spi_sck=10,
        spi_miso=11,
        spi_mosi=12,
        spi_nss=15,
        pin_dac_1=4,
        pin_adc_0=0,
        pin_adc_1=1,
        pin_adc_2=2,
        pin_adc_3=3,
        pin_adc_4=4,
        pin_adc_5=5,
        pin_adc_6=6,
        pin_adc_7=7,
        pin_adc_8=0,
        pin_adc_9=1;


gpio gpio_stm32f405;
usart uart_1;
char buffer[100]; // Буфер для хранения строки
double volt=0;
double temp_int=0,temp_ext=0,temp_rad=0;


double _TransferFunction(double voltage)
            {
            return (-1481.96 + sqrt((2.1962 * pow(10, 6)) + (1.8639 - voltage) / (3.88 * pow(10, -6))));
            }




/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM14_Init(void);

void ADC_SCAN (void);

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
	ctr=0;
	DATA_ADCacc=0;
	DATA_ADCacc1=0;
	DATA_ADCacc2=0;
	DATA_ADCacc3=0;
  int val_avg=100;
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
        }
        }
        for(int i=0;i<10;i++)
        {
          vol_arr[i]=vol_arr_temp[i]/val_avg;
        }

/*
		// Temperature
	ADC1->SQR3 = 0;
	ADC1->CR2 |= ADC_CR2_SWSTART;

		dt=0;
		while (dt<100)
		{
			dt++;
		}

	DATA_ADC[0] = ADC1->DR;
	DATA_ADCacc=DATA_ADCacc+DATA_ADC[0];

		// Bias1
	ADC1->SQR3 = 1;//3;//0;
	ADC1->CR2 |= ADC_CR2_SWSTART;

		dt=0;
		while (dt<100)
		{
			dt++;
		}

	DATA_ADC[1] = ADC1->DR;//dacc2dhr
		DATA_ADCacc1=DATA_ADCacc1+DATA_ADC[1];

		// Temperature
	ADC1->SQR3 = 2;//7;//1;
	ADC1->CR2 |= ADC_CR2_SWSTART;
	//	gcnt = 0;
	//while ((!(ADC1->SR & ADC_SR_EOC))&&(gcnt<2)) ;
		dt=0;
		while (dt<100)
		{
			dt++;
		}
	DATA_ADC[2] = ADC1->DR;
		DATA_ADCacc2=DATA_ADCacc2+DATA_ADC[2];
			// Temperature
	ADC1->SQR3 = 6;//7;//1;
	ADC1->CR2 |= ADC_CR2_SWSTART;
	//	gcnt = 0;
	//while ((!(ADC1->SR & ADC_SR_EOC))&&(gcnt<2)) ;
		dt=0;
		while (dt<100)
		{
			dt++;
		}
	DATA_ADC[3] = ADC1->DR;
		DATA_ADCacc3=DATA_ADCacc3+DATA_ADC[3];	*/
	// 	ctr++;
	// }
	DATA_ADCaccout=DATA_ADCacc/512;
	DATA_ADCaccout1=DATA_ADCacc1/512;
	DATA_ADCaccout2=DATA_ADCacc2/512;
	DATA_ADCaccout3=DATA_ADCacc3/512;
	if (DATA_ADCaccout2<4900)//0bb2 0x1200 5200
	{
  gpio_stm32f405.set_pin_state(GPIOB,led_red,1);
  gpio_stm32f405.set_pin_state(GPIOB,pin_ready,0);
  gpio_stm32f405.set_pin_state(GPIOB,led_green,0);
	// LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_11); //red
	// LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_9); //ready
	// LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_10); //green
	}
	else
	{
  gpio_stm32f405.set_pin_state(GPIOB,led_red,0);
  gpio_stm32f405.set_pin_state(GPIOB,pin_ready,1);
  gpio_stm32f405.set_pin_state(GPIOB,led_green,1);
	// LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_11); //red
	// LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_9); //ready
	// LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_10); //green
	}
	//DATA_ADC =257;
//	ADC1->SQR3 = 2;
//	ADC1->CR2 |= ADC_CR2_SWSTART;
//	while ((ADC1->SR & ADC_SR_EOC)) ;
//	DATA_ADC[1] = ADC1->DR;

//  ADC1->SQR3 = 3;
//	ADC1->CR2 |= ADC_CR2_SWSTART;
//	while ((ADC1->SR & ADC_SR_EOC)) ;
//	DATA_ADC[2] = ADC1->DR;

//	ADC1->SQR3 = 4;
//	ADC1->CR2 |= ADC_CR2_SWSTART;
//	while (!(ADC1->SR & ADC_SR_EOC)) ;
//	DATA_ADC[9] = ADC1->DR;	
//	
//	ADC1->SQR3 = 8;
//	ADC1->CR2 |= ADC_CR2_SWSTART;
//	while (!(ADC1->SR & ADC_SR_EOC)) ;
//	DATA_ADC[6] = ADC1->DR;
	
  temp_int=_TransferFunction(vol_arr[2]);
  temp_ext=_TransferFunction(vol_arr[6]*2);
  temp_rad=_TransferFunction(vol_arr[9]);
   /* sprintf(buffer, "%f", temp_int);
    uart_1.uart_tx_data(buffer);
    uart_1.uart_tx_data("/");
  volt=0;*/
}
/*
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

  while(LL_RCC_HSI_IsReady() != 1)
  {
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {
  }
  LL_Init1msTick(16000000);
  LL_SetSystemCoreClock(16000000);
}*/
void SystemClock_Config(void)
{
  // Установка задержки Flash
  FLASH->ACR &= ~FLASH_ACR_LATENCY;
  while ((FLASH->ACR & FLASH_ACR_LATENCY) != 0)
  {
  }

  // Включение HSI
  RCC->CR |= RCC_CR_HSION;
  while ((RCC->CR & RCC_CR_HSIRDY) == 0)
  {
  }

  // Установка предделителей шин
  RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);

  // Установка источника системного тактового сигнала
  RCC->CFGR &= ~RCC_CFGR_SW;
  RCC->CFGR |= RCC_CFGR_SW_HSI;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
  {
  }

  // Инициализация SysTick
  SysTick->LOAD = 16000000 - 1;
  SysTick->VAL = 0;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

  // Установка SystemCoreClock
  SystemCoreClock = 16000000;
}
/*
static void MX_ADC1_Init(void)
{

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  ADC1 GPIO Configuration
  // PA0-WKUP   ------> ADC1_IN0
  // PA1   ------> ADC1_IN1
  // PA2   ------> ADC1_IN2
  // PA3   ------> ADC1_IN3
  // PA6   ------> ADC1_IN6
  // PA7   ------> ADC1_IN7
  // PB0   ------> ADC1_IN8
  // PB1   ------> ADC1_IN9

  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_2|LL_GPIO_PIN_3|LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_REG_SetFlagEndOfConversion(ADC1, LL_ADC_REG_FLAG_EOC_UNITARY_CONV);
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV2;
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);

  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_0, LL_ADC_SAMPLINGTIME_3CYCLES);
 ADC1->CR2 |= ADC_CR2_ADON; 
}*/

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
  ADC1->CR1 &= ~ADC_CR1_RES;
  ADC1->CR2 &= ~ADC_CR2_ALIGN;
  ADC1->CR1 &= ~ADC_CR1_SCAN;
  ADC1->CR2 &= ~ADC_CR2_CONT;
  ADC1->CR2 &= ~ADC_CR2_DMA;
  ADC1->CR2 |= ADC_CR2_EOCS;
  ADC->CCR &= ~ADC_CCR_MULTI;
  ADC->CCR |= ADC_CCR_ADCPRE;

  /* ADC Channel Configuration */
  ADC1->SQR3 &= ~ADC_SQR3_SQ1;
  ADC1->SMPR2 &= ~ADC_SMPR2_SMP0;
  ADC1->SMPR2 |= ADC_SMPR2_SMP0_0 | ADC_SMPR2_SMP0_1;

  /* ADC Enable */
  ADC1->CR2 |= ADC_CR2_ADON;
}
/*
static void MX_DAC_Init(void)
{
  LL_DAC_InitTypeDef DAC_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_DAC1);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  // DAC GPIO Configuration
  // PA4   ------> DAC_OUT1
  // PA5   ------> DAC_OUT2
  
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4|LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  DAC_InitStruct.TriggerSource = LL_DAC_TRIG_SOFTWARE;
  DAC_InitStruct.WaveAutoGeneration = LL_DAC_WAVE_AUTO_GENERATION_NONE;
  DAC_InitStruct.OutputBuffer = LL_DAC_OUTPUT_BUFFER_ENABLE;
  LL_DAC_Init(DAC, LL_DAC_CHANNEL_1, &DAC_InitStruct);


  LL_DAC_Init(DAC, LL_DAC_CHANNEL_2, &DAC_InitStruct);
  DAC -> CR |= (DAC_CR_EN1)|(DAC_CR_EN2);
}*/
void MX_DAC_Init(void)
{
  /* Peripheral clock enable */
  RCC->APB1ENR |= RCC_APB1ENR_DACEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

  /* GPIO Configuration */
  //GPIOA->MODER |= GPIO_MODER_MODER4 | GPIO_MODER_MODER5;

  /* DAC Configuration */
  DAC->CR &= ~DAC_CR_TSEL1;
  DAC->CR &= ~DAC_CR_WAVE1;
  DAC->CR |= DAC_CR_BOFF1;

  /* DAC Channel 2 Configuration */
  DAC->CR &= ~DAC_CR_TSEL2;
  DAC->CR &= ~DAC_CR_WAVE2;
  DAC->CR |= DAC_CR_BOFF2;

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
/*
static void MX_SPI2_Init(void)
{
  LL_SPI_InitTypeDef SPI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  // SPI2 GPIO Configuration
  // PB12   ------> SPI2_NSS
  // PB13   ------> SPI2_SCK
  // PB14   ------> SPI2_MISO
  // PB15   ------> SPI2_MOSI
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12|LL_GPIO_PIN_13|LL_GPIO_PIN_14|LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_HARD_OUTPUT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI2, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI2, LL_SPI_PROTOCOL_MOTOROLA);
  }*/

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
/*
static void MX_TIM14_Init(void)
{

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM14);
  NVIC_SetPriority(TIM8_TRG_COM_TIM14_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);
  TIM_InitStruct.Prescaler = 0x0008;//0x8000;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 0x1000;//65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM14, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM14);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_FROZEN;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM14, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM14, LL_TIM_CHANNEL_CH1);

	//TIM14->CR1=0x81;
  //TIM14->CCMR1=0x30;	
	//TIM14->CCER=1;
	//TIM14->BDTR=0xA000;
		TIM14->CR1=0x81;
    TIM14->PSC=0x8000;	// 0x0400
	  TIM14->DIER=0x0001;
}*/
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
  TIM14->CR1 |= TIM_CR1_CEN;
}

void MX_TIM3_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Enable TIM3 clock
    TIM3->PSC = 16 - 1; // Set prescaler to 16 (SystemCoreClock = 16MHz)
    TIM3->ARR = 50000 - 1; // Set auto-reload to 10
    TIM3->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; // Set output compare 3 mode to PWM1
    TIM3->CCER |= TIM_CCER_CC3E; // Enable the output for channel 3
    TIM3->CCR3 = 44000; // Set the duty cycle to 50%
    TIM3->CR1 |= TIM_CR1_CEN; // Start TIM3
}


double PID,temp_val,temp_current,current_error,last_error,
kp=1,ki,kd,P,I,D;
double interval=5;

void pid()
{
temp_val=temp_int;
P=temp_val-temp_current;
I+=P*interval;
D=(P-last_error)/interval;
last_error=P;
PID=(kp*P+ki*I+kd*D);
}

static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);

  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13|LL_GPIO_PIN_14|LL_GPIO_PIN_0|LL_GPIO_PIN_1
                          |LL_GPIO_PIN_3|LL_GPIO_PIN_4|LL_GPIO_PIN_5|LL_GPIO_PIN_6
                          |LL_GPIO_PIN_7);
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_10|LL_GPIO_PIN_11|LL_GPIO_PIN_6
                          |LL_GPIO_PIN_8|LL_GPIO_PIN_9);// |LL_GPIO_PIN_5 |LL_GPIO_PIN_7
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_11|LL_GPIO_PIN_12);
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13|LL_GPIO_PIN_14|LL_GPIO_PIN_0|LL_GPIO_PIN_1
                          |LL_GPIO_PIN_3|LL_GPIO_PIN_4|LL_GPIO_PIN_5|LL_GPIO_PIN_6
                          |LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2|LL_GPIO_PIN_8|LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10|LL_GPIO_PIN_11|LL_GPIO_PIN_6
                          |LL_GPIO_PIN_8|LL_GPIO_PIN_9; // LL_GPIO_PIN_5||LL_GPIO_PIN_7
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_11|LL_GPIO_PIN_12|LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

int main()
{


  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  SystemClock_Config();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_SPI2_Init();
 //MX_SPI3_Init();
  MX_TIM14_Init();
  MX_TIM3_Init();
LL_GPIO_SetOutputPin(GPIOA,LL_GPIO_PIN_11); //M0 SET
  

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
//ADC
gpio_stm32f405.gpio_init(GPIOA,pin_dac_1,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_analog,gpio_speed::gpio_speed_very_high,gpio_pull_up_down::no_pull_up_down);
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


uart_1.usart_init();





	rx1[4]=32;
	rx0[0]=170;
	rx0[1]=60;
	rx0[2]=0;//7 6 6
	rx0[3]=0xe0;//32; //55 159 213 245 240
	rx0[4]=7;//10;//2
	rx0[5]=64;//104;
	rx0[6]=1;//2;//4   2 1 2 mag- 3/190 gib- 2/137
	rx0[7]=52;//170;  //158 145 169 144 148 164 132 168 138
	rx0[8]=0;	//128


/*LL_DAC_ConvertData12RightAligned (DAC1, LL_DAC_CHANNEL_1, rx0[7]+256*rx0[6]);
	LL_DAC_ConvertData12RightAligned (DAC1, LL_DAC_CHANNEL_2, rx0[5]+256*rx0[4]);*/

  DAC->DHR12R1 = rx0[7] + 256 * rx0[6];
  DAC->DHR12R2 = rx0[5] + 256 * rx0[4];

  LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_6); //PROG_B reSET
	LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_6); //PROG_B SET
	//	LL_SPI_Enable(SPI2);


uart_1.uart_tx_data("Start Programm");
uart_1.uart_tx_data("BreakPoint_1");

//USART1->DR=myString;

  while (1)
  {
ADC_SCAN ();
 //pid();
   /* sprintf(buffer, "DATA_ADC[0]: %d", DATA_ADC[0]);
                uart_1.uart_tx_data(buffer);*/
		/*rx1_s[0]=rx1[0];
		if ((DATA_ADCaccout2<=7000)&&(DATA_ADCaccout2>=3000))
		{
		rx1_s[1]=temp[DATA_ADCaccout2/2-1500]>>8;//DATA_ADCaccout2>>8;
		rx1_s[2]=temp[DATA_ADCaccout2/2-1500]&255;//DATA_ADCaccout2&255;
		}
		else
		{
			if (DATA_ADCaccout2>7000)
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

		rx1_s[3]=rx1[3];
		rx1_s[4]=DATA_ADCaccout1>>8;
		rx1_s[5]=DATA_ADCaccout1&255;
		rx1_s[6]=1;
		rx1_s[7]=3;
		rx1_s[8]=temp[DATA_ADCaccout3/2-1500]>>8;//DATA_ADCaccout3>>8;
		rx1_s[9]=temp[DATA_ADCaccout3/2-1500]&255;//DATA_ADCaccout3&255;
		rx1_s[10]=1;
		rx1_s[11]=10;
		rx1_s[12]=1;
		rx1_s[13]=0;*/
  }
}

      extern "C" void USART1_IRQHandler(void)
      {
   /* if (USART1->SR & USART_SR_TXE) // Проверка флага TXE
    {
        USART1->DR = TxData[TxCounter++];// & 0xFF; // Отправка данных
        if (TxCounter > TxData.size())
        {
            USART1->CR1 &= ~USART_CR1_TXEIE; // Отключение прерывания TXE
        }
    }*/

    if (USART1->SR & USART_SR_RXNE) // Проверка флага RXNE
    {
        RxBuffer[RxCounter++] = USART1->DR; // Прием данных
             uart_1.uart_tx_byte(USART1->DR);
             uart_1.uart_tx_byte(10);//\n
             char value_T='T';
             if(USART1->DR==value_T)
             {
                sprintf(buffer, "DATA_ADC[0]: %d", DATA_ADC[0]);
                uart_1.uart_tx_data(buffer);
                sprintf(buffer, "DATA_ADC[1]: %d", DATA_ADC[1]);
                uart_1.uart_tx_data(buffer);
                sprintf(buffer, "DATA_ADC[2]: %d", DATA_ADC[2]);
                uart_1.uart_tx_data(buffer);
                sprintf(buffer, "DATA_ADC[3]: %d", DATA_ADC[3]);
                uart_1.uart_tx_data(buffer);
             }
    }
}

void Error_Handler(void)
{

  __disable_irq();
  while (1)
  {
  }

}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
int k=0;
k++;
}
#endif /* USE_FULL_ASSERT */