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
#include "heartrate.h"

// Define Constants
#define FOSC 9830400                        // Clock frequency = Oscillator freq.
#define BAUD 9600                           // UART0 baud rate
#define MYUBRR FOSC/16/BAUD-1               // Value for UBRR0 register
#define BDIV (FOSC / 100000 - 16) / 2 + 1   // Puts I2C rate just below 100kHz
#define ADDR  0x3A
#define ADDR2 0x78
#define LOOP_CYCLES 8                       // Your clock speed in Hz (3Mhz here)

// Define Aliases/Commands


// Number of cycles that the loop takes
#define us(num) (num/(LOOP_CYCLES*(1/(FOSC/1000000.0))))

// Define Global Variables
volatile unsigned long timer_ticks = 0;

// Define Function Prototypes

