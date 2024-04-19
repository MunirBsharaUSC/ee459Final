
#include <avr/io.h>
#include <string.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <stdlib.h> // For atof() function

#define ADC_MUX_BITS 0b1111

#define ADC_PRSC  0b111         // Set the prescalar to divide by 128
// Find divisors for the UART0 and I2C baud rates
#define FOSC 7.37286e6          // Clock frequency = Oscillator freq.
#define BAUD 9600               // UART0 baud rate
#define MYUBRR 47 // FOSC/16/BAUD-1   // Value for UBRR0 register
#define BDIV (FOSC / 100000 - 16) / 2 + 1    // Puts I2C rate just below 100kHz
#define ADDR  0x3A
#define ADDR2 0x78
#define LCD_CLEAR_DISPLAY 0x01
#define LCD_RETURN_HOME 0x02
#define LCD_ENTRY_MODE_SET 0x06
#define LCD_DISPLAY_ON 0x0F // Display on, cursor off, blink off
#define LCD_FUNCTION_SET 0x38 // 4-bit mode, 2 lines, 5x8 dots


uint8_t i2c_io(uint8_t, uint8_t *, uint16_t, uint8_t *, uint16_t);
void i2c_init(unsigned char);
unsigned char USART_Receive(void);
void USART_Transmit(unsigned char);
void USART_Init(unsigned int);
void lcd_init(void);
void lcd_send_command(uint8_t);
void lcd_send_data(char *);
void parse_gpgga(const char *data, char *latitude, char *longitude);


#define LOOP_CYCLES      8       //Your clock speed in Hz (3Mhz here)
#define BEAT_THRESHOLD 200 // Adjust this based on your sensor data range

#define us(num) (num/(LOOP_CYCLES*(1/(FOSC/1000000.0))))

#define MAX_BUFFER_SIZE 128
char buffer[MAX_BUFFER_SIZE];
volatile uint8_t buffer_index = 0;
volatile uint8_t data_ready = 0;

ISR(USART_RX_vect) {
    char received_char = UDR0; 
    if (received_char == '\n') {
        buffer[buffer_index] = '\0'; 
        buffer_index = 0; 
        data_ready = 1; 
    } else {
        buffer[buffer_index++] = received_char; 
        if (buffer_index >= MAX_BUFFER_SIZE - 1) {
            buffer_index = 0; 
        }
    }
}

int main(){
    USART_Init(MYUBRR);
    i2c_init(BDIV);
    lcd_init();
    lcd_send_data("GPS DATA");
    _delay_ms(1000);
    lcd_send_command(LCD_CLEAR_DISPLAY);

    sei();
    
    char longitude[20];
    char latitude[20];

    while (1) {
        if (data_ready) {
            // lcd_send_data(buffer);
            parse_gpgga(buffer, latitude, longitude);
            _delay_ms(1000);
            
            data_ready = 0;
            lcd_send_command(LCD_CLEAR_DISPLAY);
        }
     }
}

void parse_gpgga(const char *data, char *latitude, char *longitude) {
    
    char gpsBuffer[20];
    char gpsBuffer1[20];
    char *token;
    int count = 0;

    char data_copy[strlen(data) + 1];
    strcpy(data_copy, data);

    token = strtok(data_copy, ",");
    while (token != NULL) {
        count++;
        if (count == 3) {
            strcpy(latitude, token);
        }
        else if (count == 5) {
            strcpy(longitude, token);
            break;
        }
        token = strtok(NULL, ",");
    }
    sprintf(gpsBuffer, "Latitude %s\n", latitude);
    lcd_send_data(gpsBuffer);
    
    sprintf(gpsBuffer1, "Longitude %s", longitude);
    lcd_send_data(gpsBuffer1);

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

void USART_Init(unsigned int ubrr)
{
    UBRR0 = ubrr;         // Set baud rate
    UCSR0C = (3 << UCSZ00);     // Set for asynchronous operation, no parity,
                                // one stop bit, 8 data bits
    UCSR0B |= (1 << TXEN0);     // Turn on transmitter
    UCSR0B |= (1 << RXEN0);     // Turn on receiver
    UCSR0B |= (1 << RXCIE0);    // Enable receiver interrupts
    DDRD |= (1 << PD1);
}

uint8_t i2c_io(uint8_t device_addr, uint8_t *wp, uint16_t wn, uint8_t *rp, uint16_t rn)
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

void i2c_init(uint8_t bdiv)
{
    TWSR = 0;                           // Set prescalar for 1
    TWBR = bdiv;                        // Set bit rate register
}
