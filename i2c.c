/*********************************************************************
*       i2c - I/O routines for the ATmega168 TWI interface,
*       which is the same functionally as I2C.
*
*       Note: These routines were written to help students in EE459Lx
*       at USC.  They may contain errors, and definitely are not
*       ready for use in a real product.  Use at your own risk.
*
* Revision History
* Date     Author      Description
* 04/14/11 A. Weber    First release
* 02/07/12 A. Weber    Added i2c_write and i2c_read1 routines
* 02/07/12 A. Weber    Added i2c_write1 routine
* 02/17/12 A. Weber    Changes to comments and types
* 04/19/12 A. Weber    Combined write and read routines into one function
* 05/08/12 A. Weber    Added code to handle NAKs better
* 04/09/15 A. Weber    More comments
* 01/07/24 A. Weber    Changed i2c_io to only use two buffers
*********************************************************************/

#include <avr/io.h>
#include <string.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>

#define ADC_MUX_BITS 0b1111

#define ADC_PRSC  0b111         // Set the prescalar to divide by 128
// Find divisors for the UART0 and I2C baud rates
#define FOSC 9830400            // Clock frequency = Oscillator freq.
#define BAUD 9600               // UART0 baud rate
#define MYUBRR FOSC/16/BAUD-1   // Value for UBRR0 register
#define BDIV (FOSC / 100000 - 16) / 2 + 1    // Puts I2C rate just below 100kHz
#define ADDR  0x3A
#define ADDR2 0x78
#define LCD_CLEAR_DISPLAY 0x01
#define LCD_RETURN_HOME 0x02
#define LCD_ENTRY_MODE_SET 0x06
#define LCD_DISPLAY_ON 0x0F // Display on, cursor off, blink off
#define LCD_FUNCTION_SET 0x38 // 4-bit mode, 2 lines, 5x8 dots
#include <time.h>
uint8_t i2c_io(uint8_t, uint8_t *, uint16_t, uint8_t *, uint16_t);
void i2c_init(unsigned char);
void adc_init(void);
uint8_t adc_sample(uint8_t);
unsigned char USART_Receive(void);
void USART_Transmit(unsigned char);
void USART_Init(unsigned int);
void lcd_init(void);
void lcd_send_command(uint8_t);
uint8_t therm_reset(void);
void therm_write_bit(uint8_t);
uint8_t therm_read_byte(void);
uint8_t therm_read_bit(void);
void therm_write_byte(uint8_t);
void therm_read_temperature(char *);
void lcd_send_data(char *);

#define THERM_PORT PORTC
#define THERM_DDR DDRC
#define THERM_PIN PINC
#define THERM_DQ PC3
#define THERM_DECIMAL_STEPS_12BIT 625
#define THERM_INPUT_MODE() THERM_DDR&=~(1<<THERM_DQ)
#define THERM_OUTPUT_MODE() THERM_DDR|=(1<<THERM_DQ)
#define THERM_LOW() THERM_PORT&=~(1<<THERM_DQ)
#define THERM_HIGH() THERM_PORT|=(1<<THERM_DQ)
#define LOOP_CYCLES      8       //Your clock speed in Hz (3Mhz here)
#define BEAT_THRESHOLD 200 // Adjust this based on your sensor data range
#define MAX_BEATS 10 // Number of beats to average for heart rate calculation
// Number of cycles that the loop takes
#define us(num) (num/(LOOP_CYCLES*(1/(FOSC/1000000.0))))
uint8_t smooth_signal(uint8_t);
void heartbeatCalc(char *);
void timer1_init();
unsigned long millis();
inline __attribute__((gnu_inline)) void therm_delay(uint16_t delay){ while(delay--) asm volatile("nop"); }
int main(void) {

    uint8_t wdata[2];              // Buffer for reading data from EEPROM
    uint8_t rdata[6];              // Buffer for reading data from EEPROM
    i2c_init(BDIV);

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
            for(int i=0; i<MAX_BEATS;i++){
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
uint8_t therm_reset(){
    uint8_t i;
    THERM_LOW();
    THERM_OUTPUT_MODE();
    _delay_us(480);
    //therm_delay(us(480));

    THERM_INPUT_MODE();
    _delay_us(60);
    //therm_delay(us(60));

    i=(THERM_PIN & (1<<THERM_DQ));
    _delay_us(420);
    //therm_delay(us(420));

    return i;
}

void therm_write_bit(uint8_t bit){

    THERM_LOW();
    THERM_OUTPUT_MODE();
    _delay_us(1);
    //therm_delay(us(1));

    if(bit) THERM_INPUT_MODE();

    _delay_us(60);
    //therm_delay(us(60));
    THERM_INPUT_MODE();
}

uint8_t therm_read_bit(void){
    uint8_t bit=0;

    THERM_LOW();
    THERM_OUTPUT_MODE();
    _delay_us(1);

    THERM_INPUT_MODE();
    _delay_us(14);
    //therm_delay(us(14));

    if(THERM_PIN&(1<<THERM_DQ)) bit=1;

    _delay_us(45);
    //therm_delay(us(45));
    return bit;
}

uint8_t therm_read_byte(void){
    uint8_t i=8, n=0;

    while(i--){
        n>>=1;
        n|=(therm_read_bit()<<7);
    }

    return n;
}

void therm_write_byte(uint8_t byte){
    uint8_t i=8;

    while(i--){
        therm_write_bit(byte&1);
        byte>>=1;
    }
}

#define THERM_CMD_SKIPROM 0xcc
#define THERM_CMD_CONVERTTEMP 0x44
#define THERM_CMD_RSCRATCHPAD 0xbe
void therm_read_temperature(char *buffer){
    uint8_t temperature[2];
    int8_t digit;
    uint16_t decimal;

    therm_reset();
    therm_write_byte(THERM_CMD_SKIPROM);
    therm_write_byte(THERM_CMD_CONVERTTEMP);

    while(!therm_read_bit());

    therm_reset();
    therm_write_byte(THERM_CMD_SKIPROM);
    therm_write_byte(THERM_CMD_RSCRATCHPAD);

    temperature[0]=therm_read_byte();
    temperature[1]=therm_read_byte();
    therm_reset();

    digit=temperature[0]>>4;
    digit|=(temperature[1]&0x7)<<4;

    decimal=temperature[0]&0xf;
    decimal*=THERM_DECIMAL_STEPS_12BIT;

    sprintf(buffer, "%+d.%04u C", digit, decimal);

}
void lcd_send_command(uint8_t command) {
    uint8_t control_byte = 0x80; // Control byte to indicate a command (adjust based on your LCD's protocol)
    uint8_t command_packet[2] = {control_byte, command};

    // Send the command using i2c_io; adjust the device_addr as necessar
    i2c_io(ADDR2, command_packet, 2, NULL, 0);
}

void lcd_send_data(char *command) {
    uint8_t control_byte = 0x40; // Control byte to indicate a command (adjust based on your LCD's protocol)
    uint8_t command_packet[strlen(command)+1];
    command_packet[0]=control_byte;
    uint8_t i=strlen(command);

    while(i!=0){
        command_packet[strlen(command)-i+1]=command[strlen(command)-i];
        i--;
    }

    // Send the command using i2c_io; adjust the device_addr as necessar
    i2c_io(ADDR2, command_packet, strlen(command)+1, NULL, 0);
}


void lcd_init(void) {
    _delay_ms(500); // Wait for LCD to power up

    // Function Set: configure the display
    lcd_send_command(LCD_FUNCTION_SET);
    _delay_us(120); // Wait for command to execute

    // Display ON/OFF Control: turn on the display without cursor or blinking
    lcd_send_command(LCD_DISPLAY_ON);
    _delay_us(120); // Wait for command to execute

    // Clear Display: clear the display and set cursor position to zero
    lcd_send_command(LCD_CLEAR_DISPLAY);
    _delay_ms(15); // Wait for command to execute

    // Entry Mode Set: set the text entry mode (adjust as necessary)
    lcd_send_command(LCD_ENTRY_MODE_SET);
    _delay_us(120); // Wait for command to execute
}
void USART_Init(unsigned int ubrr){
    // Set baud rate
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)ubrr;
    // Enable receiver and transmitter
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);
    // Set frame format: 8 data bits, 1 stop bit
    UCSR0C = (3<<UCSZ00);
}

void USART_Transmit(unsigned char data){
    // Wait for empty transmit buffer
    while ( !( UCSR0A & (1<<UDRE0)) );
    // Put data into buffer, sends the data
    UDR0 = data;
}

unsigned char USART_Receive(void){
    // Wait for data to be received
    while ( !(UCSR0A & (1<<RXC0)) );
    // Get and return received data from buffer
    return UDR0;
}

/*int main(void){
    USART_Init(MYUBRR);

    while(1){
        unsigned char receivedChar = USART_Receive();
        USART_Transmit(receivedChar); // Echo received data
    }
}*/
void adc_init(void)
{
    // Initialize the ADC
    ADMUX |= (1 << REFS0);   // Set the REFS bits
    ADMUX |= (1 << ADLAR);      // Left adjust the output
    ADCSRA |= (ADC_PRSC << ADPS0);  // Set the prescalar bits
    ADCSRA |= (1 << ADEN);      // Enable the ADC
}

uint8_t adc_sample(uint8_t channel)
{
    ADMUX &= ~ADC_MUX_BITS;
    ADMUX |= ((channel & ADC_MUX_BITS) << MUX0); // Set the MUX bits
    ADCSRA |= (1 << ADSC);      // Start a conversion
    while (ADCSRA & (1 << ADSC)); // wait for conversion complete
    return ADCH;                // Get converted value
}
/*
  i2c_io - write and read bytes to an I2C device

  Usage:      status = i2c_io(device_addr, wp, wn, rp, rn);
  Arguments:  device_addr - This is the EIGHT-BIT I2C device bus address.
              Some datasheets specify a seven bit address.  This argument
              is the seven-bit address shifted left one place with a zero
              in the LSB.  This is also sometimes referred to as the
              address for writing to the device.
              wp, rp - Pointers to two buffers containing data to be
              written (wp), or to receive data that is read (rp).
              wn, rn - Number of bytes to write or read to/from the
              corresponding buffers.

  This funtions writes "wn" bytes from array "wp" to I2C device at
  bus address "device_addr".  It then reads "rn" bytes from the same device
  to array "rp".

  Return values (might not be a complete list):
        0    - Success
        0x20 - NAK received after sending device address for writing
        0x30 - NAK received after sending data
        0x38 - Arbitration lost with address or data
        0x48 - NAK received after sending device address for reading

  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

This "i2c_io" I2C routine is an attempt to provide an I/O function for both
reading and writing, rather than have separate functions.

I2C writes consist of sending a stream of bytes to the slave device.  In some
cases the first few bytes may be the internal address in the device, and then
the data to be stored follows.  For example, EEPROMs like the 24LC256 require a
two-byte address to precede the data.  The DS1307 RTC requires a one-byte
address.

I2C reads often consist of first writing one or two bytes of internal address
data to the device and then reading back a stream of bytes starting from that
address.  Some devices appear to claim that that reads can be done without
first doing the address writes, but so far I haven't been able to get any to
work that way.

This function does writing and reading by using pointers to two arrays
"wp", and "rp".  The function performs the following actions in this order:
    If "wn" is greater then zero, then "wn" bytes are written from array "wp"
    If "rn" is greater then zero, then "rn" byte are read into array "rp"
Either of the "wn" or "rn" can be zero.

A typical write with a 2-byte address and 50 data bytes is done with

    i2c_io(0xA0, wbuf, 52, NULL, 0);

A typical read of 20 bytes with a 1-byte address is done with

    i2c_io(0xD0, wbuf, 1, rbuf, 20);
*/

uint8_t i2c_io(uint8_t device_addr,
               uint8_t *wp, uint16_t wn, uint8_t *rp, uint16_t rn)
{
    uint8_t status, send_stop, wrote, start_stat;

    status = 0;
    wrote = 0;
    send_stop = 0;

    if (wn > 0) {
        wrote = 1;
        send_stop = 1;

        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA);  // Send start condition
        while (!(TWCR & (1 << TWINT)));     // Wait for TWINT to be set
        status = TWSR & 0xf8;
        if (status != 0x08)                 // Check that START was sent OK
            return(status);

        TWDR = device_addr & 0xfe;          // Load device address and R/W = 0;
        TWCR = (1 << TWINT) | (1 << TWEN);  // Start transmission
        while (!(TWCR & (1 << TWINT)));     // Wait for TWINT to be set
        status = TWSR & 0xf8;
        if (status != 0x18) {               // Check that SLA+W was sent OK
            if (status == 0x20)             // Check for NAK
                goto nakstop;               // Send STOP condition
            return(status);                 // Otherwise just return the status
        }

        // Write "wn" data bytes to the slave device
        while (wn-- > 0) {
            TWDR = *wp++;                   // Put next data byte in TWDR
            TWCR = (1 << TWINT) | (1 << TWEN); // Start transmission
            while (!(TWCR & (1 << TWINT))); // Wait for TWINT to be set
            status = TWSR & 0xf8;
            if (status != 0x28) {           // Check that data was sent OK
                if (status == 0x30)         // Check for NAK
                    goto nakstop;           // Send STOP condition
                return(status);             // Otherwise just return the status
            }
        }

        status = 0;                         // Set status value to successful
    }

    if (rn > 0) {
        send_stop = 1;

        // Set the status value to check for depending on whether this is a
        // START or repeated START
        start_stat = (wrote) ? 0x10 : 0x08;

        // Put TWI into Master Receive mode by sending a START, which could
        // be a repeated START condition if we just finished writing.
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA);
                                            // Send start (or repeated start) condition
        while (!(TWCR & (1 << TWINT)));     // Wait for TWINT to be set
        status = TWSR & 0xf8;
        if (status != start_stat)           // Check that START or repeated START sent OK
            return(status);

        TWDR = device_addr  | 0x01;         // Load device address and R/W = 1;
        TWCR = (1 << TWINT) | (1 << TWEN);  // Send address+r
        while (!(TWCR & (1 << TWINT)));     // Wait for TWINT to be set
        status = TWSR & 0xf8;
        if (status != 0x40) {               // Check that SLA+R was sent OK
            if (status == 0x48)             // Check for NAK
                goto nakstop;
            return(status);
        }

        // Read all but the last of n bytes from the slave device in this loop
        rn--;
        while (rn-- > 0) {
            TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA); // Read byte and send ACK
            while (!(TWCR & (1 << TWINT))); // Wait for TWINT to be set
            status = TWSR & 0xf8;
            if (status != 0x50)             // Check that data received OK
                return(status);
            *rp++ = TWDR;                   // Read the data
        }

        // Read the last byte
        TWCR = (1 << TWINT) | (1 << TWEN);  // Read last byte with NOT ACK sent
        while (!(TWCR & (1 << TWINT)));     // Wait for TWINT to be set
        status = TWSR & 0xf8;
        if (status != 0x58)                 // Check that data received OK
            return(status);
        *rp++ = TWDR;                       // Read the data

        status = 0;                         // Set status value to successful
    }

nakstop:                                    // Come here to send STOP after a NAK
    if (send_stop)
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);  // Send STOP condition

    return(status);
}


/*
  i2c_init - Initialize the I2C port
*/
void i2c_init(uint8_t bdiv)
{
    TWSR = 0;                           // Set prescalar for 1
    TWBR = bdiv;                        // Set bit rate register
}
