#include <avr/io.h>
#include <string.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <stdlib.h> // For atof() function

#define MYUBRR 47 // FOSC/16/BAUD-1   // Value for UBRR0 register

#define MAX_BUFFER_SIZE 128
char buffer[MAX_BUFFER_SIZE];
volatile uint8_t buffer_index = 0;
volatile uint8_t data_ready = 0;

char longitude[20]; 
char latitude[20];

extern void parse_gpgga(const char *data, char *latitude, char *longitude);
extern bool isGPSLocked(const char *buffer[], int *length);
