// Standard Include Libraries
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <string.h>
#include <util/delay.h>

// Include File Dependencies
#include "i2c.h"
#include "usart.h"
#include "adc.h"
#include "lcd.h"
#include "therm.h"

// Define Constants
#define ADC_MUX_BITS 0b1111
#define ADC_PRSC  0b111         // Set the prescalar to divide by 128
#define FOSC 9830400            // Clock frequency = Oscillator freq.
#define BAUD 9600               // UART0 baud rate
#define MYUBRR FOSC/16/BAUD-1   // Value for UBRR0 register
#define BDIV (FOSC / 100000 - 16) / 2 + 1    // Puts I2C rate just below 100kHz
#define ADDR  0x3A
#define ADDR2 0x78
#define THERM_CMD_SKIPROM 0xcc
#define THERM_CMD_CONVERTTEMP 0x44
#define THERM_CMD_RSCRATCHPAD 0xbe
#define LCD_CLEAR_DISPLAY 0x01
#define LCD_RETURN_HOME 0x02
#define LCD_ENTRY_MODE_SET 0x06
#define LCD_DISPLAY_ON 0x0F // Display on, cursor off, blink off
#define LCD_FUNCTION_SET 0x38 // 4-bit mode, 2 lines, 5x8 dots
#define LOOP_CYCLES      8       //Your clock speed in Hz (3Mhz here)
#define BEAT_THRESHOLD 200 // Adjust this based on your sensor data range
#define MAX_BEATS 10 // Number of beats to average for heart rate calculation

// Define Aliases/Commands
#define THERM_PORT PORTC
#define THERM_DDR DDRC
#define THERM_PIN PINC
#define THERM_DQ PC3
#define THERM_DECIMAL_STEPS_12BIT 625
#define THERM_INPUT_MODE() THERM_DDR&=~(1<<THERM_DQ)
#define THERM_OUTPUT_MODE() THERM_DDR|=(1<<THERM_DQ)
#define THERM_LOW() THERM_PORT&=~(1<<THERM_DQ)
#define THERM_HIGH() THERM_PORT|=(1<<THERM_DQ)
// Number of cycles that the loop takes
#define us(num) (num/(LOOP_CYCLES*(1/(FOSC/1000000.0))))

// Function Prototypes
uint8_t smooth_signal(uint8_t);
void heartbeatCalc(char *);
void timer1_init();
unsigned long millis();
inline __attribute__((gnu_inline)) void therm_delay(uint16_t delay){ while(delay--) asm volatile("nop"); }
