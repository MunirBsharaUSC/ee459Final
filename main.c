// Include Dependencies
#include "main.h"

// Main Function
int main(void) {

    // Initialize I2C and LCD (for boot message)
    i2c_init(BDIV);
    _delay_ms(100); // Wait for i2c initialization
    lcd_init();

    // Display welcome/initialization message
    lcd_clear(0);
    lcd_print("   Welcome to the   ", 1);
    lcd_print("  SummitWand v1.0!  ", 2);
    _delay_ms(1000);
    lcd_print("    The Enhanced    ", 3);
    lcd_print(" Hiking Experience! ", 4);
    _delay_ms(2000);
    lcd_clear(3);
    lcd_print("Initializing Boot...", 4);
    _delay_ms(2000);

    // Initialization
    USART_Init(MYUBRR);
    adc_init();
    therm_reset();
    timer1_init();
    sei();

    // Variables and Buffers
    uint8_t wdata[2];              // Buffer for reading data from EEPROM
    uint8_t rdata[6];              // Buffer for reading data from EEPROM

    // Display succesful boot
    lcd_clear(3);
    lcd_clear(4);
    lcd_print("   Boot Succesful!  ", 4);
    _delay_ms(2000);
    lcd_clear(0);

    // Main Program Loop
    while(1){
        lcd_print("In main program loop now...", 0);
    };

    return 0;   // Never reached
}
