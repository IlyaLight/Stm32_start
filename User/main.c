#include "stm32f10x.h"                  // Device header 
#define F_CPU 			72000000UL
#define APB1_F_CPU 	F_CPU/2


void USART2_IRQHandler(void){
	if(USART2->SR & USART_SR_RXNE){
		USART2->DR += 2;
	}
}

void USART3_IRQHandler(void){
	if(USART3->SR & USART_SR_RXNE){
		USART3->DR += 3;
	}
}


int main(void)
{
	SystemInit();
	/*----------------------------- 
		обевление переменых					,компилятор не пропускает объявление в не начале блока({})
	-----------------------------*/
	
	
	
	
	__IO uint32_t StarUpCounter = 0, HSEStatus = 0;		//"__IO" -тип памяти в которой храниться переменная
	
	/*-----------------------------
		настройка переферии контролера
	-----------------------------*/
	RCC->APB2ENR  |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN |	// GPIOA Clock ON. Alter function clock ON
		 RCC_APB2ENR_IOPBEN;
	
	
	//----USART----
#define BAUD_RATE 	9600

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	//USART
	
	GPIO_InitTypeDef GIP_UART;	// экземпляр структуры с нстройками порта
	GIP_UART.GPIO_Speed = GPIO_Speed_10MHz;
	
	//USART2
	//set tx
	GIP_UART.GPIO_Pin 	= GPIO_Pin_2;			//12-USART2_TX	(USART2_TX/ADC12_IN2/TIM2_CH3)
	GIP_UART.GPIO_Mode 	= GPIO_Mode_AF_PP;
	GPIO_Init (GPIOA, &GIP_UART);
	//set rx
	GIP_UART.GPIO_Pin 	= GPIO_Pin_3;			//13-USART2_RX	(USART2_RX/ADC12_IN3/TIM2_CH4)
	GIP_UART.GPIO_Mode 	= GPIO_Mode_IPU;
	GPIO_Init (GPIOA, &GIP_UART);
	//set USART2 reg
	RCC->APB1ENR 	|= RCC_APB1ENR_USART2EN;	//тактируем модуль USART2
	USART2->BRR 	= (APB1_F_CPU+(BAUD_RATE>>1) )/ BAUD_RATE;	//прописываем битрэйд http://we.easyelectronics.ru/STM32/nastroyka-uart-v-stm32-i-problemy-dvoichno-desyatichnoy-arifmetiki.html
	USART2->CR1		|= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;	//USART2 ON, TX ON, RX ON, RXNE ON
	
	//USART3
	//set tx
	//GIP_UART.GPIO_Pin 	= GPIO_Pin_10;			//12-USART3_TX	(USART2_TX/ADC12_IN2/TIM2_CH3)
	//GIP_UART.GPIO_Mode 	= GPIO_Mode_AF_PP;
	//GPIO_Init (GPIOB, &GIP_UART);
	//set rx
	//GIP_UART.GPIO_Pin 	= GPIO_Pin_11;			//13-USART3_RX	(USART2_RX/ADC12_IN3/TIM2_CH4)
	//GIP_UART.GPIO_Mode 	= GPIO_Mode_IPU;
	//GPIO_Init (GPIOA, &GIP_UART);
	//set USART3 reg
	//RCC->APB1ENR	|= RCC_APB1ENR_USART3EN;	//тактируем модуль USART3
	//USART2->BRR 	= (APB1_F_CPU+(BAUD_RATE>>1) )/ BAUD_RATE;	//прописываем битрэйд http://we.easyelectronics.ru/STM32/nastroyka-uart-v-stm32-i-problemy-dvoichno-desyatichnoy-arifmetiki.html
	//USART3->CR1		|= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;	//USART2 ON, TX ON, RX ON, RXNE ON
	
	//настройка портов
	//RCC->APB2ENR	|= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//тактируем порт A
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	
//	RCC->APB2ENR|=RCC_APB2ENR_IOPCEN;
//	GPIOC->CRH|=0x44144444;
	
	
	//led
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitTypeDef GPIO_Init_LED;
	GPIO_Init_LED.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init_LED.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init_LED.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_Init_LED);
	
	//NVIC_EnableIRQ(USART2_IRQn);
	//NVIC_EnableIRQ(USART3_IRQn);
	
	//__enable_irq();		//включаем глобальные прерывания 
	uint32_t i;
	while(1)
	{
		
		GPIO_SetBits(GPIOC, GPIO_Pin_13);
		//GPIOC->ODR=0x0000;
		for(i=0;i<5000000;i++){}
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		//GPIOC->ODR=0xf000;
		for(i=0;i<500000;i++){}
		GPIO_SetBits(GPIOC, GPIO_Pin_13);
		//GPIOC->ODR=0x0000;
		for(i=0;i<500000;i++){}
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		//GPIOC->ODR=0xf000;
		for(i=0;i<500000;i++){}
		USART2->DR = 11;
	}		
}

