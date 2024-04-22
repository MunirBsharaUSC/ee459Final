#include "pulse.h"

extern volatile unsigned long timer_ticks;

// Simple moving average filter for smoothing
uint8_t smooth_signal(uint8_t new_sample) {
    static int samples[10];
    static int index = 0;
    static int sum = 0;
    const int sample_count = sizeof(samples) / sizeof(samples[0]);

    sum -= samples[index];
    samples[index] = new_sample;
    sum += samples[index];

    index = (index + 1) % sample_count;

    return sum / sample_count;
}

void timer1_init() {
    // Configure the timer for normal mode (counter increases until it overflows back to 0)
    // Stop the timer
    TCCR1B &= ~((1 << CS10) | (1 << CS11) | (1 << CS12));

    TCNT1 = 0; // Reset timer count
    TCCR1B |= (1 << CS10) | (1 << CS11); // Set up timer with prescaler = 64
    TCNT1 = 0; // Initialize counter
    TIMSK1 |= (1 << TOIE1); // Enable overflow interrupt
}

// Timer1 overflow interrupt service routine
// This ISR is called every time Timer1 overflows
ISR(TIMER1_OVF_vect) {
        // Increment our timer tick count
        timer_ticks++;
}

// Function to get current time in milliseconds
unsigned long millis() {
    unsigned long millis_ret;
    cli();
    unsigned long total_ticks = timer_ticks * 65536 + TCNT1;
    millis_ret = (total_ticks * 10) / 768;
    TCNT1=0;
    timer_ticks=0;
    sei();
    return millis_ret;
}

void heartbeatCalc(char *buffer, unsigned long *count, unsigned long *beat_times, uint8_t *currIndex, uint8_t *startIndex, uint8_t *en, uint8_t *prevSample) {
    uint8_t sample;

    unsigned long countTemp=*count;
    uint8_t currIndexTemp=*currIndex;
    uint8_t startIndexTemp=*startIndex;
    uint8_t enTemp=*en;
    sample = adc_sample(2);
    if ((sample >= BEAT_THRESHOLD) && (*prevSample<BEAT_THRESHOLD)){
        countTemp=0;
        beat_times[currIndexTemp]=millis();

        //lcd_send_data("3");
        //_delay_ms(275);
        //_delay_ms(25);
        currIndexTemp = (currIndexTemp+1)%MAX_BEATS;
        *currIndex = currIndexTemp;
        if(currIndexTemp==startIndexTemp){
            //lcd_send_data("4");
            //_delay_ms(25);
            enTemp=1;
            startIndexTemp=(startIndexTemp+1)%MAX_BEATS;
            *startIndex=startIndexTemp;
        }
    }
    *prevSample=sample;
    countTemp++;
    if(countTemp==300){
        uint8_t i;
        for(i=0; i<MAX_BEATS; i++){
            beat_times[i]=0;
        }
        *startIndex=0;
        *currIndex=0;
        enTemp=0;
        countTemp=1;
        //lcd_send_command(LCD_CLEAR_DISPLAY);
        //_delay_ms(15);
    }
    if(!enTemp){
        snprintf(buffer, 20, "      NO BEATS      ");
    }
    else if(enTemp && countTemp==1){
        unsigned long total=0;
        uint8_t i;
        for(i=0; i<MAX_BEATS; i++){
            if(i!=startIndexTemp){
            total += beat_times[i];}
        }
        total = (MAX_BEATS * 60000) /(total);
        snprintf(buffer, 20, "     %3d BEATS     ", (int)total);
   /*     lcd_send_command(LCD_CLEAR_DISPLAY);
        _delay_ms(15);
        lcd_send_data(buffer);*/
    }
    *count=countTemp;
    *en=enTemp;
}
