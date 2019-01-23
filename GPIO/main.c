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

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	//Configure LED Pin
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = 	GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = 	GPIO_Pin_10; 	
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	while (1) {

		GPIO_SetBits(GPIOA,GPIO_Pin_10);

		Delay_1us(1);
		GPIO_ResetBits(GPIOA,GPIO_Pin_10);

		Delay_1us(2);


	}
}
