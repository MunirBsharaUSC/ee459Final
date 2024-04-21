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

    // Initialization
    USART_Init(MYUBRR);
    adc_init();
    accel_init();
    therm_reset();
    timer1_init();
    sei();

    // Variables and Buffers
    char accel_buf[20];
    int16_t x, y, z;

    // Display succesful boot
    _delay_ms(1000);
    lcd_clear(3);
    lcd_clear(4);
    lcd_print("   Boot Succesful!  ", 4);
    _delay_ms(1000);
    lcd_clear(0);

    // Main Program Loop (accelerometer test)
    while(1){
        accel_read(&x, &y, &z);
        lcd_print("ACCELEROMETER", 1);
        snprintf(accel_buf, 20, "X:%d", x);
        lcd_clear(2);
        lcd_print(accel_buf, 2);
        snprintf(accel_buf, 20, "Y:%d", y);
        lcd_clear(3);
        lcd_print(accel_buf, 3);
        snprintf(accel_buf, 20, "Z:%d", z);
        lcd_clear(4);
        lcd_print(accel_buf, 4);

        _delay_ms(100);
    };

    return 0;   // Never reached
}
