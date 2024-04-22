#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>

#include "adc.h"

#define BEAT_THRESHOLD 240 // Adjust this based on your sensor data range
#define MAX_BEATS 10 // Number of beats to average for heart rate calculation

extern uint8_t smooth_signal(uint8_t);
extern void heartbeatCalc(char *, unsigned long *, unsigned long *, uint8_t *, uint8_t *,  uint8_t *, uint8_t *);
extern void timer1_init();
extern unsigned long millis();
