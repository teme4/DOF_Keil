#ifndef _GPIO
#define _GPIO
#include "usart.hpp"
#include "stm32f4xx.h"
#include <cmath>
#include "stdio.h"
#include <string.h>

uint8_t len=0;
// uint8_t TxBuffer[100];//="start programm";
// uint8_t RxBuffer[100];
// uint8_t TxCounter = 0, RxCounter = 0;

    /**********************************************************************************
     * @brief Конфигураяция Baud rate
     *********************************************************************************/
      uint16_t usart::GetBBRsettings(uint32_t ClokFreq,float baurd_rate)
      {
        double UsarDiv=static_cast<double>(ClokFreq)/(baurd_rate);
        return roundf(UsarDiv);
      }

     /**********************************************************************************
     * @brief Конфигураяция USART
     *********************************************************************************/
      void usart::usart_init()
      {
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN; 
        USART1->CR3  = 0;
        USART1->CR2 =  0;
        USART1->CR1  = 0;
        USART1->BRR =  GetBBRsettings(SystemCoreClock,115200);
        USART1->CR1 =USART_CR1_UE | USART_CR1_TE | USART_CR1_RE| USART_CR1_RXNEIE ; 
       // USART1->CR1 |=USART_CR1_TCIE;     
        NVIC_EnableIRQ(USART1_IRQn);
        __enable_irq();
      }
     /**********************************************************************************
     * @brief Отправка одного байта или массива байт
     *********************************************************************************/
      void usart::uart_tx_byte(uint8_t  data)
      {
      while ((USART1->SR & USART_SR_TXE) == 0)  {}//USART_SR_TXE
      USART1->DR = data;
      } 
      
      void usart::uart_tx_data(const char *data)
      {
      len = strlen(data); 
      while(len--)
      {
      usart::uart_tx_byte(*data++);
      }
      usart::uart_tx_byte(10);//\n
      }
//     /**********************************************************************************
//      * @brief Прием и передача данных прерывания
//      *********************************************************************************/
//       extern "C" void USART1_IRQHandler(void)
// {
//     if (USART1->SR & USART_SR_TXE) // Проверка флага TXE
//     {
//         USART1->DR = TxBuffer[TxCounter++];// & 0xFF; // Отправка данных

//         if (TxCounter > sizeof(TxBuffer))
//         {
//             USART1->CR1 &= ~USART_CR1_TXEIE; // Отключение прерывания TXE
//         }
//     }

//     if (USART1->SR & USART_SR_RXNE) // Проверка флага RXNE
//     {
//         RxBuffer[RxCounter++] = USART1->DR; // Прием данных
//     }
// }
#endif //_GPIO