

#ifndef __USART_RXTX_H
#define __USART_RXTX_H

void USART_NVIC_Configuration(void);
void USART_GPIO_Configuration(void);
void USART_Configuration(void);
void USART1_IRQHandler(void);
void UARTSend(const unsigned char *pucBuffer, unsigned long ulCount);

#endif
