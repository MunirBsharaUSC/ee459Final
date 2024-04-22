// Standard Include Libraries
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <string.h>
#include <util/delay.h>

// Include File Dependencies
#include "i2c.h"
#include "usart.h"
#include "adc.h"
#include "lcd.h"
#include "therm.h"
#include "accel.h"
#include "pulse.h"
#include "gps.h"
#include "button.h"
#include "timer.h"

// Define Constants
#define FOSC 7.37286e6  //9830400                        // Clock frequency = Oscillator freq.
#define BAUD 9600                           // UART0 baud rate
#define MYUBRR 47                   // FOSC/16/BAUD-1 Value for UBRR0 register
#define BDIV (FOSC / 100000 - 16) / 2 + 1   // Puts I2C rate just below 100kHz
#define LOOP_CYCLES 8                       // Your clock speed in Hz (3Mhz here)

// Define Aliases/Commands/States
#define STATE_ERROR     -1
#define STATE_HOME       0
#define STATE_GPS        1
#define STATE_PULSE      2
#define STATE_TEMP       3
#define STATE_ACCEL      4
#define STATE_TRIP       5
#define STATE_TRIP_RESET 6
#define NUM_STATES       6

// Number of cycles that the loop takes
#define us(num) (num/(LOOP_CYCLES*(1/(FOSC/1000000.0))))

// Declare Global Variables
volatile int8_t state;
volatile int8_t state_change;
volatile uint8_t button_hold_count;
volatile unsigned long timer_ticks;
volatile unsigned long trip_time;
volatile uint8_t timer_update;

volatile uint8_t gps_data_ready;
volatile char gps_buffer[128];
char latitude[20];
char longitude[20];
char dir1[2];
char dir2[2];
