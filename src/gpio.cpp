#include "gpio.hpp"


/*
#define no_pull_up_down						0
#define pull_up					         	0x01
#define pull_down					        0x10*/

 void gpio::gpio_init(GPIO_TypeDef * GPIOx, uint32_t pin,gpio_type type, gpio_mode mode,gpio_speed speed,gpio_pull_up_down pull_up_down)
  {
  RCC->AHB1ENR |= 1<<(((uint32_t)GPIOx - AHB1PERIPH_BASE) / 0x400);
  GPIOx-> MODER &=~(3<<(pin*2U));//сбросить
  GPIOx-> MODER|=(uint8_t)mode <<(pin*2U);//прописать
  GPIOx->OTYPER|=(uint8_t)type<<(pin);
  GPIOx->OSPEEDR|=(uint8_t)speed<<(pin);  
  GPIOx->PUPDR|=(uint8_t)pull_up_down;
  }

 void  gpio::set_pin_state(GPIO_TypeDef *GPIOx,uint8_t pin,uint8_t state)
  {
  if(state==1)
  {
   GPIOx->ODR |= (1<<pin);
  }
   if(state==0)
  {
    GPIOx->ODR&= ~ (1<<pin);
  } 
  } 

int gpio::get_state_pin (GPIO_TypeDef *GPIOx, uint8_t pin)
  {
	  uint16_t mask;
	  mask = ( 1<< pin);
   	if ((GPIOx->IDR) & mask) return 1;
	  else return 0;
  }

  /**********************************************************************************
     * @brief Конфигураяция пина для альтернативной функции 
     * *для STM32F4
     *********************************************************************************/
     void gpio::config_af(GPIO_TypeDef *GPIOx, uint8_t PIN, uint8_t AF) 
    {
        if (PIN > 7){
            GPIOx->AFR[1] |= AF << (4 * (PIN - 8)); //AFRH
        } else {
            GPIOx->AFR[0] |= (AF << (4 * PIN));     //AFRL
        }
    }