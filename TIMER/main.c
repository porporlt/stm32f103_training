#include "stm32f10x.h"
#include "stm32f10x_conf.h"

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
	GPIO_InitStructure.GPIO_Mode = 	GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = 	GPIO_Pin_1; 	
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//Possible output modes are:
	// GPIO_Mode_Out_OD        ;output open drain
	// GPIO_Mode_Out_PP        ;output push-pull
	// GPIO_Mode_AF_OD         ;alternate function open drain
	// GPIO_Mode_AF_PP         ;alternate function push pull
	// IMPORTANT: The first 2 are meant for main function IO pins. If Alternate function IO pin is used, the later 2 should be used. E.g. all UART transmit output pins must configure as AF output pin.
	// Refer to section 3 on a useful general IO pin configuration function.







	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);


	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 14400-1;
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)(1-1);
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_OCInitTypeDef TIM_OCInitStruct;

	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	//TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStruct.TIM_Pulse = 2160-1;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	//TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCPolarity_High;
	//TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
	//TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

	TIM_OC2Init(TIM2, &TIM_OCInitStruct);
	TIM_Cmd(TIM2, ENABLE);

	uint16_t cnt = 14400-1;

	//int timerValue = TIM_GetCounter(TIM2);

	while (1) {

		GPIO_SetBits(GPIOC,GPIO_Pin_13);

		/* Set timer CCR2 register (pulse) directly */
		TIM2 -> CCR2 = cnt;
		cnt-- ;
		if(cnt == 0){
			//while(cnt > 0){
			//	cnt-- ;
			//	Delay_1us(100);
			//}
			cnt = 14400-1;
		}
		

		//Delay_1us(100000);
		//GPIO_ResetBits(GPIOC,GPIO_Pin_13);
		Delay_1us(100);

	}

}
