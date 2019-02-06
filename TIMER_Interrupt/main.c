#include "stm32f10x.h"
#include "stm32f10x_conf.h"

void TIM3_IRQHandler(void);
void gpio_toggle(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

//status
int status = 0 ; 


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

void TIM3_IRQHandler()
{
        if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET){ // clear flag (TIM_IT_Update)
                //gpio_toggle(GPIOA, GPIO_Pin_0);
                status = status+1;
                if (status <= 2){
                // gpio_toggle(GPIOC, GPIO_Pin_13);
                TIM2 -> CCR2 = 1000;
                }
                if (status == 3){
                TIM2 -> CCR2 = 0;
                status = 0;
            	}

                TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
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


	// ADD AF PWM PIN
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


	// NVIC_InitTypeDef NVIC_InitStructure;

	// /* Enable the TIM2 global Interrupt */
	// NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	// NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // set interrupt group
	// NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	// NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	// NVIC_Init(&NVIC_InitStructure);

	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the TIM3 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // set interrupt group
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);



	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 2000-1;
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)(720-1);
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);


	TIM_TimeBaseInitTypeDef TIM3_TimeBaseStructure;
	/* Time base configuration */
	TIM3_TimeBaseStructure.TIM_Period = 20000-1;
	TIM3_TimeBaseStructure.TIM_Prescaler = (uint16_t)(720-1);
	TIM3_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM3_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM3_TimeBaseStructure);



	// ADD OUTPUT COMPARE PWM
	TIM_OCInitTypeDef TIM_OCInitStruct;

	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStruct.TIM_Pulse = 1000-1;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

	TIM_OC2Init(TIM2, &TIM_OCInitStruct);


	// TIM_ITConfig( TIM2, TIM_IT_Update, ENABLE); // TIM_IT_Update = interrupt when นับครบ (สามเหลี่ยม)
	TIM_ITConfig( TIM3, TIM_IT_Update, ENABLE); // TIM_IT_Update = interrupt when นับครบ (สามเหลี่ยม)
	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM3, ENABLE);

	while (1) {

	}
}
