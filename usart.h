#include <avr/interrupt.h>

extern unsigned char USART_Receive(void);
extern void USART_Transmit(unsigned char);
extern void USART_Init(unsigned int);
