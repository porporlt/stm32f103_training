#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include <stdio.h>
#include <stdlib.h>

void init_Tim2(void);
void init_usart1(void);
void RTC_Init(void);
void send_byte(uint8_t b);
void usart_puts(char* s);
void TIM2_IRQHandler(void);
void gpio_toggle(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

static inline void Delay_1us(uint32_t nCnt_1us)
{
  volatile uint32_t nCnt;

  for (; nCnt_1us != 0; nCnt_1us--)
    for (nCnt = 13; nCnt != 0; nCnt--);
}

static inline void Delay(uint32_t nCnt_1us)
{

			while(nCnt_1us--);
}

void gpio_toggle(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIOx->ODR ^= GPIO_Pin;
}

void TIM2_IRQHandler()
{
        if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){ // clear flag (TIM_IT_Update)

                gpio_toggle(GPIOA, GPIO_Pin_1);
                gpio_toggle(GPIOC, GPIO_Pin_13);

                TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        }
}

void RTC_Init()
	{	
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
		PWR_BackupAccessCmd(ENABLE);
		

		// Disable RTC && Reset counter
		RCC_RTCCLKCmd(DISABLE);
		// RCC_BackupResetCmd(ENABLE); 
		// RCC_BackupResetCmd(DISABLE);

		if ((RCC->BDCR & RCC_BDCR_RTCEN) != RCC_BDCR_RTCEN) //RTCEN:RTC clock enable
		{
		
		RCC_BackupResetCmd(ENABLE); 
		RCC_BackupResetCmd(DISABLE);
		//BDRST: Backup domain software reset 
		//Set and cleared by software
		//0: Reset not activated
		//1: Resets the entire Backup domain


		RCC_LSEConfig(RCC_LSE_ON);// 32.768KHz
		while ((RCC->BDCR & RCC_BDCR_LSERDY) != RCC_BDCR_LSERDY) {} //LSERDY:External low-speed oscillator ready
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

		RTC_SetPrescaler(0x7FFF); // 1HZ set (32768/32768)

		RCC_RTCCLKCmd(ENABLE);
		RTC_WaitForSynchro();

		}
	}

void init_usart1()
{

	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable peripheral clocks. */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* Configure USART1 Rx pin as floating input. */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART1 Tx as alternate function push-pull. */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure the USART1 */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // set interrupt group
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable transmit and receive interrupts for the USART1. */
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
	// USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	USART_Cmd(USART1, ENABLE);

}

void init_Tim2(){

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the TIM3 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // set interrupt group
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 10000-1;
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)(7200-1);
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	// TIM_OCInitTypeDef TIM_OCInitStruct;

	// TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	// TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	// TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable;
	// TIM_OCInitStruct.TIM_Pulse = 50-1;
	// TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	// TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCPolarity_High;
	// TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
	// TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

	// TIM_OC2Init(TIM2, &TIM_OCInitStruct);
	TIM_ITConfig( TIM2, TIM_IT_Update, ENABLE); 
	TIM_Cmd(TIM2, ENABLE);

}


void send_byte(uint8_t b)
{
	/* Send one byte */
	USART_SendData(USART1, b);

	/* Loop until USART2 DR register is empty */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}


void usart_puts(char* s)
{
    while(*s) {
    	send_byte(*s);
        s++;
    }
}


int main(void)
{
	/* Sawasdee OH */

	GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	//Configure LED Pin
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = 	GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = 	GPIO_Pin_13; 	
	GPIO_Init(GPIOC, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	//Configure LED Pin
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = 	GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = 	GPIO_Pin_1; 	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//Possible output modes are:
	// GPIO_Mode_Out_OD        ;output open drain
	// GPIO_Mode_Out_PP        ;output push-pull
	// GPIO_Mode_AF_OD         ;alternate function open drain
	// GPIO_Mode_AF_PP         ;alternate function push pull
	// IMPORTANT: The first 2 are meant for main function IO pins. If Alternate function IO pin is used, the later 2 should be used. E.g. all UART transmit output pins must configure as AF output pin.
	// Refer to section 3 on a useful general IO pin configuration function.


	// RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
	// RCC_HCLKConfig(RCC_SYSCLK_Div512);	

	uint32_t RTC_Counter = 0;
	char buffer[80] = {'\0'};

	init_Tim2();
	init_usart1();
    RTC_Init();
    
	while (1) {

	RTC_Counter = RTC_GetCounter();
	sprintf(buffer, "\r\n\r\nCOUNTER: %d\r\n", (int)RTC_Counter);
	usart_puts(buffer);
	Delay_1us(100000);

	}

}
