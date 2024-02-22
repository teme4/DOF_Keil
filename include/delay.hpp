#ifndef _DELAY_
#define _DELAY_

#include "stm32f4xx.h"

#define    DWT_CYCCNT    *(volatile unsigned long *)0xE0001004
#define    DWT_CONTROL   *(volatile unsigned long *)0xE0001000
#define    SCB_DEMCR     *(volatile unsigned long *)0xE000EDFC


void sys_delay_us(uint32_t us);
void sys_delay_ms(uint32_t ms);

 #endif //_DELAY_