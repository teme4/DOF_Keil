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
unsigned short DATA_ADC[4];
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
        spi_nss=15;


gpio gpio_stm32f405;
usart uart_1;
char buffer[100]; // Буфер для хранения строки
double volt=0;
double temp_int=0,temp_ext=0;





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
	while (ctr<1024)
	{
		// Temperature
	ADC1->SQR3 = 0;//2;
	ADC1->CR2 |= ADC_CR2_SWSTART;
//		gcnt = 0;
	//while ((!(ADC1->SR & ADC_SR_EOC))&&(gcnt<2)) ;
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
	//	gcnt = 0;
	//while ((!(ADC1->SR & ADC_SR_EOC))&&(gcnt<2)) ;
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
		DATA_ADCacc3=DATA_ADCacc3+DATA_ADC[3];	
		ctr++;
	}
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
	
  volt=DATA_ADC[2]*(3.3/4096);
  temp_int=sqrt(2196200+((1.8639-volt)/0.00000388))-1481.96;
  //ext temp
  volt=DATA_ADC[3]*(3.3/4096);
  temp_ext=sqrt(2196200+((1.8639-volt)/0.00000388))-1481.96;
  volt=0;
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
}


static void MX_ADC1_Init(void)
{

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**ADC1 GPIO Configuration
  PA0-WKUP   ------> ADC1_IN0
  PA1   ------> ADC1_IN1
  PA2   ------> ADC1_IN2
  PA3   ------> ADC1_IN3
  PA6   ------> ADC1_IN6
  PA7   ------> ADC1_IN7
  PB0   ------> ADC1_IN8
  PB1   ------> ADC1_IN9
  */
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
}


static void MX_DAC_Init(void)
{
  LL_DAC_InitTypeDef DAC_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_DAC1);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**DAC GPIO Configuration
  PA4   ------> DAC_OUT1
  PA5   ------> DAC_OUT2
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4|LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  DAC_InitStruct.TriggerSource = LL_DAC_TRIG_SOFTWARE;
  DAC_InitStruct.WaveAutoGeneration = LL_DAC_WAVE_AUTO_GENERATION_NONE;
  DAC_InitStruct.OutputBuffer = LL_DAC_OUTPUT_BUFFER_ENABLE;
  LL_DAC_Init(DAC, LL_DAC_CHANNEL_1, &DAC_InitStruct);

  /** DAC channel OUT2 config*/
  LL_DAC_Init(DAC, LL_DAC_CHANNEL_2, &DAC_InitStruct);
  DAC -> CR |= (DAC_CR_EN1)|(DAC_CR_EN2);
}


static void MX_SPI2_Init(void)
{
  LL_SPI_InitTypeDef SPI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**SPI2 GPIO Configuration
  PB12   ------> SPI2_NSS
  PB13   ------> SPI2_SCK
  PB14   ------> SPI2_MISO
  PB15   ------> SPI2_MOSI*/
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
uart_1.usart_init();

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  SystemClock_Config();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM14_Init();
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
	rx0[8]=0;	//128
	LL_DAC_ConvertData12RightAligned (DAC1, LL_DAC_CHANNEL_1, rx0[7]+256*rx0[6]);
	LL_DAC_ConvertData12RightAligned (DAC1, LL_DAC_CHANNEL_2, rx0[5]+256*rx0[4]);
	//LL_DAC_ConvertData12RightAligned (DAC2, LL_DAC_CHANNEL_1, rx0[3]+256*rx0[2]);	
  LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_6); //PROG_B reSET
	LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_6); //PROG_B SET
	//	LL_SPI_Enable(SPI2);


uart_1.uart_tx_data("Start Programm");
uart_1.uart_tx_data("BreakPoint_1");

//USART1->DR=myString;

  while (1)
  {

ADC_SCAN ();

		rx1_s[0]=rx1[0];
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
		rx1_s[13]=0;
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