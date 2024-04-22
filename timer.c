#include "timer.h"

extern volatile uint16_t trip_time;
uint8_t overflow_ticks;

void timer2_init(void){
    TCCR2A = 0;                             // Normal mode
    TCCR2B &= ~((1 << WGM22) | (1 << WGM21) | (1 << WGM20)); // Normal mode
    TCNT2 = 0;                              // Reset count
    TIMSK2 |= (1 << TOIE2);                 // Enable timer overflow interrupt
    TCCR0B |= (1 << CS02) | (1 << CS00);    // Set prescalar to 1024 (enable timer)
    overflow_ticks = 0;
}

void timer2_disable(void){
    TCCR2B = 0;
    TIMSK2 &= ~(1 << TOIE2);                 // Enable timer overflow interrupt
}

ISR(TIMER2_OVF_vect) {
    overflow_ticks++;
    if(overflow_ticks >= 38){
        overflow_ticks = 0;
        trip_time++;
    }
}