// Include Dependencies
#include "main.h"

// Main Function
int main(void) {

    // Component Initialization
    i2c_init(BDIV);
    USART_Init(MYUBRR);
    adc_init();
    therm_reset();
    lcd_init();
    lcd_send_command(LCD_CLEAR_DISPLAY);
    timer1_init();
    sei();

    // Variables and Buffers
    uint8_t wdata[2];              // Buffer for reading data from EEPROM
    uint8_t rdata[6];              // Buffer for reading data from EEPROM
    //uint8_t x;                   // Unused
    memset(wdata, 0, 2);
    wdata[0]=0x31;
    wdata[1]=0x00;
    //x = i2c_io(ADDR,wdata, 2, NULL,0);
    //_delay_ms(10);
    memset(wdata, 0, 2);memset(rdata, 0, 6);
    wdata[0]=0x2D;
    wdata[1]=0x08;
    //x = i2c_io(ADDR,wdata, 2, NULL, 0);
    //_delay_ms(10);
    memset(wdata, 0, 2);memset(rdata, 0, 6);
    wdata[0]=0x31;
    wdata[1]=0x0B;
    //x = i2c_io(ADDR,wdata, 2, NULL, 0);
    //_delay_ms(10);
    char buffer[30];
    char text[5];
    memset(text, 0, 5);
    text[0]='J';
    text[1]='D';
    text[2]='J';
    text[3]='D';
    text[4]='J';

    // Main Program Loop
    while(1) {

        lcd_send_command(LCD_CLEAR_DISPLAY);
        _delay_ms(15);
        memset(wdata, 0, 2);
        memset(buffer, 0, 15);
        _delay_ms(100);
        therm_reset();
        //sprintf(buffer,"%d",adc_sample(2));
        heartbeatCalc(buffer);
        therm_read_temperature(buffer);
        text[1]=buffer[0];
        text[2]=buffer[1];
        text[3]=buffer[2];
        text[4]=buffer[3];
        text[5]=buffer[4];
        //Write the control byte followed by the text to the LCD
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
