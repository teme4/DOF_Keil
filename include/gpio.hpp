#pragma once //исходный файл при компиляции подключался строго один раз


#include "stm32f4xx.h"

enum class gpio_pull_up_down //PUDR
 {
    no_pull_up_down =      0,                
    pull_up=               1,                    
    pull_down =            2        
};
enum class gpio_mode //MODERN
 {
    gpio_mode_input =     0,                
    gpio_mode_output=     1,                    
    gpio_mode_alternate = 2,                  
    gpio_mode_analog =    3                     
};
enum class gpio_type //OTYPER
 {
    gpio_type_pp =          0,                
    gpio_type_od=           1           
 };
enum class gpio_speed //OSPEEDR
 {
    gpio_speed_low =          0,                
    gpio_speed_medium=        1,      
    gpio_speed_high =         2,                
    gpio_speed_very_high=     3     
 };

class gpio
{
private: 
 
public:
 // void gpio_init(GPIO_TypeDef * port, uint32_t pin,gpio_type type, gpio_mode mode);  
  void gpio_init(GPIO_TypeDef * GPIOx, uint32_t pin,gpio_type type, gpio_mode mode,gpio_speed speed,gpio_pull_up_down pull_up_down);
  void set_pin_state(GPIO_TypeDef *GPIOx,uint8_t pin,uint8_t state);
  int get_state_pin (GPIO_TypeDef *GPIOx, uint8_t pin);
  void config_af(GPIO_TypeDef *GPIOx, uint8_t PIN, uint8_t AF); 
};



