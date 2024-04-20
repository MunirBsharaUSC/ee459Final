#include <avr/interrupt.h>

#define ADC_MUX_BITS 0b1111
#define ADC_PRSC  0b111         // Set the prescalar to divide by 128

extern void adc_init(void);
extern uint8_t adc_sample(uint8_t);
