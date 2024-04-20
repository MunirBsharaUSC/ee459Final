// Include Dependencies
#include "main.h"

// Main Function
int main(void) {

    uint8_t wdata[2];              // Buffer for reading data from EEPROM
    uint8_t rdata[6];              // Buffer for reading data from EEPROM
    i2c_init(BDIV);
    USART_Init(MYUBRR);

    uint8_t x;
    adc_init();
    lcd_init();
    therm_reset();
    memset(wdata, 0, 2);
    wdata[0]=0x31;
    wdata[1]=0x00;
    //x = i2c_io(ADDR,wdata, 2, NULL,0);
    _delay_ms(10);
    memset(wdata, 0, 2);memset(rdata, 0, 6);
    wdata[0]=0x2D;
    wdata[1]=0x08;
    //x = i2c_io(ADDR,wdata, 2, NULL, 0);
    _delay_ms(10);
    memset(wdata, 0, 2);memset(rdata, 0, 6);
    wdata[0]=0x31;
    wdata[1]=0x0B;
    //x = i2c_io(ADDR,wdata, 2, NULL, 0);
    _delay_ms(10);
    char buffer[30];
    char text[5];
    memset(text, 0, 5);
    text[0]='J';
    text[1]='D';
    text[2]='J';
    text[3]='D';
    text[4]='J';
    timer1_init();
    while(1) {

        lcd_send_command(LCD_CLEAR_DISPLAY);
        _delay_ms(15);
        memset(wdata, 0, 2);
        memset(buffer, 0, 15);
        _delay_ms(100);
        therm_reset();
        //sprintf(buffer,"%d",adc_sample(2));
        heartbeatCalc(buffer);
        //therm_read_temperature(buffer);
        //text[1]=buffer[0];
        //text[2]=buffer[1];
        //text[3]=buffer[2];
        //text[4]=buffer[3];
        //text[5]=buffer[4];
// Write the control byte followed by the text to the LCD
        lcd_send_data(buffer);
        //i2c_io(ADDR2, '0x40', 6, NULL, 0);

        //memset(wdata, 0, 1);memset(rdata, 0, 6);
        //wdata[0]=0x32;
        //wdata[0]=adc_sample(2); ADC2 pulse, ADC3 therm
        //x = i2c_io(ADDR,wdata, 2, rdata, 6);
        //_delay_ms(1);
        _delay_ms(2000);

    }
}


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
volatile  unsigned long timer_ticks =0;
// Initialize Timer1
void timer1_init() {
    // Configure the timer for normal mode (counter increases until it overflows back to 0)
    // Stop the timer
    TCCR1B &= ~((1 << CS10) | (1 << CS11) | (1 << CS12));

    TCNT1 = 0; // Reset timer count
    TCCR1B |= (1 << CS10) | (1 << CS11); // Set up timer with prescaler = 64
    TCNT1 = 0; // Initialize counter
    TIMSK1 |= (1 << TOIE1); // Enable overflow interrupt
    sei(); // Enable global interrupts
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
    sei();
    return millis_ret;
}
void heartbeatCalc(char *buffer) {
    int beat_count = 0;
    unsigned long last_beat_time = 0;
    unsigned long current_time;
    unsigned long beat_times[MAX_BEATS] = {0};
    int last_sample = 0;
    int current_sample;
    int count=0;
    timer_ticks=0;
    while (beat_count < MAX_BEATS) {
        current_sample = adc_sample(2);

        //current_time = millis(); // Get current time in milliseconds
// Simple threshold-based beat detection
        if (last_sample < BEAT_THRESHOLD && current_sample >= BEAT_THRESHOLD) {
            beat_times[beat_count++] = millis();
            count=0;
            //Beat detected!
        }
        count++;
        last_sample = current_sample;
        if(count==100){
            count=0;
            beat_count=0;
            last_sample=0;
            timer_ticks=0;
            int i = 0;
            for(i=0; i<MAX_BEATS;i++){
                beat_times[i] = 0;
            }
        }
        _delay_ms(30);

    }

// Calculate average heart rate
    if (beat_count == MAX_BEATS) {

        unsigned long total_beat_time = beat_times[MAX_BEATS - 1] - beat_times[0];
        unsigned long heart_rate = ((beat_count+3) * 60000) /(total_beat_time);

        sprintf(buffer, "%ld beats", heart_rate);
    }
}

/*int main(void){
    USART_Init(MYUBRR);

    while(1){
        unsigned char receivedChar = USART_Receive();
        USART_Transmit(receivedChar); // Echo received data
    }
}*/

