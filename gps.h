#include <avr/io.h>
#include <string.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <stdlib.h> // For atof() function

#include "lcd.h"

volatile uint8_t buffer_index;

extern void parse_gpgga(void);
extern char isGPSLocked(void);
extern void parse_gprmc(void);