
#include "delay.h"
#include "stm32f10x.h"

void delay_10ms(unsigned int delay)
{
     TIM2->PSC = (SystemCoreClock / 10000) - 1; /* 7200 */
     TIM2->ARR = delay;
     TIM2->EGR |= TIM_EGR_UG;
     TIM2->CR1 |= TIM_CR1_CEN | TIM_CR1_OPM;
     while ((TIM2->CR1 & TIM_CR1_CEN) != 0);
}

void delay_us(unsigned int delay)
{
     TIM2->PSC = (SystemCoreClock / 1000000)  - 1; /* 72 */
     TIM2->ARR = delay;
     TIM2->EGR |= TIM_EGR_UG;
     TIM2->CR1 |= TIM_CR1_CEN | TIM_CR1_OPM;
     while ((TIM2->CR1 & TIM_CR1_CEN) != 0);
}


	
