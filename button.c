#include "button.h"

extern volatile int8_t state;
extern volatile int8_t state_change;
extern volatile uint8_t button_hold_count;
volatile uint8_t timer_running;

void button_init(void){
    DDRD &= ~(1 << DDD2);       // Set PD2 as input (for push button)
    PORTD |= (1 << PD2);        // Enable internal pull-up resistor for PD2
    PCMSK2 |= (1 << PCINT18);   // Configure Pin Change Interrupt
    PCICR |= (1 << PCIE2);      // Configure Pin Change Interrupt
    state_change = 0;           // Reset state change flag
    button_hold_count = 0;      // Reset button hold time
    timer_running = 0;          // Reset timer running flag
}

void enable_button_interrupt(void){
    PCICR |= (1 << PCIE2);
}

void disable_button_interrupt(void){
    PCICR &= ~(1 << PCIE2);
}

void timer0_init(void){
    TCCR0A = 0;                             // Normal mode
    TCCR0B &= ~((1 << WGM02) | (1 << WGM01) | (1 << WGM00)); // Normal mode
    TCNT0 = 0;                              // Reset count
    TIMSK0 |= (1 << TOIE0);                 // Enable timer overflow interrupt
    timer_running = 1;                      // Set timer start flag
    button_hold_count = 0;                  // Reset hold count
    TCCR0B |= (1 << CS02) | (1 << CS00);    // Set prescalar to 1024 (enable timer)
}

void timer0_disable(void){
    TCCR0B = 0;
    TIMSK0 &= ~(1 << TOIE0);                 // Enable timer overflow interrupt
    timer_running = 0;
}

ISR(TIMER0_OVF_vect) {
    if(!(PIND & (1 << PD2))){  // Button still pressed
        button_hold_count++;
        if(button_hold_count >= 120){
            button_hold_count = 0;
            state = STATE_TRIP_RESET;
            timer0_disable();
        }
    }
    else{ // Button released before 3 second hold. Perform normal transition
        timer0_disable();
        state = STATE_HOME;
        state_change = 1;
    }
}

ISR(PCINT2_vect){
    if(!(PIND & (1 << PD2))){
        if(state != STATE_TRIP){
            state = (state + 1) % NUM_STATES;
            state_change = 1;
            PCICR &= ~(1 << PCIE2);
        }
        else if(state == STATE_TRIP && !timer_running){
            if(!timer_running){ // Button hold timer start
                timer0_init();
                PCICR &= ~(1 << PCIE2);
            }
        }
    }
}
