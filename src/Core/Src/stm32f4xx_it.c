/* Includes ------------------------------------------------------------------*/
#include "main.hpp"
#include "stm32f4xx_it.h"

extern volatile unsigned short DATA_ADCaccout2;
extern volatile unsigned short bias[1601];
 uint16_t spi_rx16[4];
volatile uint8_t rx1[18];
volatile uint8_t rx1_s[18];
  uint8_t rx2[10] __attribute__((used));
  uint8_t ptr=0;
  uint8_t ptr2=0;
  uint16_t baddr=700;
  int16_t bias_off=-173+26+77-58+16+64;
	//300+119-185+135+80-79-109+130-16+96;//-146-32-210+96; //-292
	uint16_t threshold = 0x12d;//0x132;//139
volatile uint8_t cntspi2=0;
volatile uint8_t cnt_v[2];
volatile uint8_t spi2_v[5];
volatile uint8_t lev;
volatile uint8_t pkuc[6];
volatile uint8_t gcnt;
int32_t StartPageAddr = 0x08008000;
uint64_t Data[6]= {0x31111, 0x42222, 0x3333, 0x4444, 0x5555, 0x6666};

void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
	TIM14->SR=0;
	ptr=0;
	gcnt++;
	/*
	rx2[0]=170;
	rx2[1]=60;
	rx2[2]=6;
	rx2[3]=155;
	rx2[4]=2;
	rx2[5]=155;
	rx2[6]=4;
	rx2[7]=0;
	*/
	
	if (rx1[8]<100)
	{
	//if ((DATA_ADCaccout<7646)&&(DATA_ADCaccout>7000))//(((DATA_ADCaccout&255)<240)&&((DATA_ADCaccout>>8)>0x0b))
	//{LL_DAC_ConvertData12RightAligned (DAC2, LL_DAC_CHANNEL_1, ((7646-(DATA_ADCaccout))>>2)+0x710);//LL_DAC_ConvertData12RightAligned (DAC1, LL_DAC_CHANNEL_1, ((240-(DATA_ADCaccout&255))>>2)+256*6);
	//}
	if ((DATA_ADCaccout2<7000)&&(DATA_ADCaccout2>=4900))//(((DATA_ADCaccout&255)<240)&&((DATA_ADCaccout>>8)>0x0b))
	{
		baddr=DATA_ADCaccout2-4899;
//		if (pkuc[0]==9)
//		{
//			LL_DAC_ConvertData12RightAligned (DAC2, LL_DAC_CHANNEL_1, (bias[baddr]+rx1[6]-256+256*pkuc[4]+pkuc[3]));
//		  LL_DAC_ConvertData12RightAligned (DAC1, LL_DAC_CHANNEL_1, 256*pkuc[2]+pkuc[1]);
//		}
//			else
		{
		LL_DAC_ConvertData12RightAligned (DAC1, LL_DAC_CHANNEL_2, (bias[baddr]+rx1[6]+bias_off));//-512+16+64               +48+32 -320 - BRI, -320+177 - slx4-2, -320+260 itmo 001, -320+147, -320+250+359 itmo 009, -320+196itmo 005, -320+480 Q?, -320+522 011
	    LL_DAC_ConvertData12RightAligned (DAC1, LL_DAC_CHANNEL_1, threshold);//48+256*1); // 44 76  72+4+256*1   176+8+256*2
		}
		// 67A@4719 det 025
	}
	else
	{ if (DATA_ADCaccout2>=7200)
		{LL_DAC_ConvertData12RightAligned (DAC1, LL_DAC_CHANNEL_2, 0x100);//0x600 0x570
		}
		else
		{LL_DAC_ConvertData12RightAligned (DAC1, LL_DAC_CHANNEL_2, 0x100);
		}
		;
	}
}
	
	//rx1[0]=70;
/*	rx1[1]=6;
	rx1[2]=7;
	rx1[3]=91;
	rx1[4]=10;//2
	rx1[5]=104;
	rx1[6]=10;//4
	rx1[7]=0;*/
	LL_GPIO_ResetOutputPin(GPIOA,LL_GPIO_PIN_15);
	LL_SPI_Enable(SPI3);	
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
			cnt_v[1]=spi_rx16[1];
			cnt_v[0]=spi_rx16[1]>>8;
			pkuc[0]=spi_rx16[2]>>12;
			pkuc[1]=spi_rx16[2]&0x00ff;
			pkuc[2]=(spi_rx16[2]>>8)&0x0f;
			pkuc[3]=spi_rx16[3]&0x00ff;
			pkuc[4]=(spi_rx16[3]>>8)&0x0f;			
 return; 
	}


/*
void USART1_IRQHandler(void)
{
 
  //  if(LL_USART_IsActiveFlag_RXNE(USART1) != 0)
  //  {
        //If received 't', toggle LED and transmit 'T' 
  rx1[ptr2]=       LL_USART_ReceiveData8(USART1);
	if(rx1[0] == 170)
	{
		ptr2++;
		if (ptr2>7)
		{
			ptr2=0;
			if (rx1[1]>=50)
			{
			ptr2=0;
			rx1[0]=0;
			LL_USART_TransmitData8(USART1,55);

			ptr2=0;
			while (ptr2<1)
			{						
				LL_USART_TransmitData8(USART1,(spi_rx16[ptr2]));
				while (LL_USART_IsActiveFlag_TXE(USART1)==0)
				{
				}
				LL_USART_TransmitData8(USART1,(spi_rx16[ptr2]>>8));
				while (LL_USART_IsActiveFlag_TXE(USART1)==0)
				{
				}					
			  ptr2++;
			}
			ptr2=0;
			while (ptr2<1)
			{						
				LL_USART_TransmitData8(USART1,(DATA_ADCaccout2));
				while (LL_USART_IsActiveFlag_TXE(USART1)==0)
				{
				}
				LL_USART_TransmitData8(USART1,(DATA_ADCaccout2>>8));
				while (LL_USART_IsActiveFlag_TXE(USART1)==0)
				{
				}					
			  ptr2++;
			}
		}
			else
		{		
			if (((rx1[3]&0x0C)^0x04) == 0x0C )
			{
				rx1[3]=0;
				rx1[0]=0;
				WRITE_REG(FLASH->KEYR,0x45670123);
				WRITE_REG(FLASH->KEYR,0xCDEF89AB);
				
			  if ((READ_BIT(FLASH->SR,FLASH_SR_BSY))==0)	
				{
					SET_BIT (FLASH->CR, FLASH_CR_SER);
					MODIFY_REG(FLASH->CR, FLASH_CR_SNB, (0x2<<FLASH_CR_SNB_Pos));
					MODIFY_REG(FLASH->CR, FLASH_CR_PSIZE, (0x2<<FLASH_CR_PSIZE_Pos));
					SET_BIT (FLASH->CR, FLASH_CR_STRT);
					while (READ_BIT(FLASH->SR,FLASH_SR_BSY)) 
					{
					}
					CLEAR_BIT(FLASH->CR, FLASH_CR_SER);
				}
			  if ((READ_BIT(FLASH->SR,FLASH_SR_BSY))==0)	
				{
					SET_BIT(FLASH->CR, FLASH_CR_PG);	
					*(uint64_t *)StartPageAddr = 0xAAAAAAAA12345678;//Data[0];
					while (READ_BIT(FLASH->SR,FLASH_SR_BSY)) 
					{
					}
					CLEAR_BIT(FLASH->CR, FLASH_CR_SER);
					SET_BIT (FLASH->CR, FLASH_CR_LOCK);
				}
				
			LL_USART_TransmitData8(USART1,57);
			}
		}	
			LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_9);
	      LL_SPI_Enable(SPI2);	
			ptr=2;
			while (ptr<12)
			{
			 LL_SPI_TransmitData16(SPI2,rx1[ptr]+256*rx1[ptr+1]);
			 
			 while (LL_SPI_IsActiveFlag_BSY(SPI2)) //LL_SPI_IsActiveFlag_RXNE(SPI2)==0
				{
				}
			 //spi_rx16[ptr/2]=LL_SPI_ReceiveData16(SPI2);
			 ptr=ptr+2;
			}

		
			LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_9);
						LL_SPI_Disable(SPI2);	
			
		
			ptr2=0;		
		}
			ptr2=0;
		}		
	}
	else
	{
		ptr2=0;
	}
 
         
       return;   
}
*/