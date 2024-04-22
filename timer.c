#include "timer.h"

extern volatile uint16_t trip_time;
uint8_t overflow_ticks;
extern volatile uint8_t timer_update;

void timer2_init(void){
    TCCR2A = 0;                             // Normal mode
    TCCR2B &= ~((1 << WGM22) | (1 << WGM21) | (1 << WGM20)); // Normal mode
    TCNT2 = 0;                              // Reset count
    TIMSK2 |= (1 << TOIE2);                 // Enable timer overflow interrupt
    overflow_ticks = 0;
    timer_update = 0;
    TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);    // Set prescalar to 1024 (enable timer)
}

void timer2_disable(void){
    TCCR2B = 0;
    TIMSK2 &= ~(1 << TOIE2);                 // Disable timer overflow interrupt
}

ISR(TIMER2_OVF_vect) {
    overflow_ticks++;
    if(overflow_ticks >= 37){
        overflow_ticks = 0;
        trip_time++;
        timer_update = 1;
    }
}