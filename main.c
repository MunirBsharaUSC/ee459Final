// Include Dependencies
#include "main.h"

volatile uint8_t buttonPressed = 0;
volatile uint8_t screenCounter = 0;

ISR(PCINT2_vect) {
    if (!(PIND & (1 << PD2))) { // Check if button is pressed
        buttonPressed = 1;
    }
}

void updateScreen() {
    switch (screenCounter) {
        case 0:
            // Display screen 1: Intro
            lcd_clear(0);
            lcd_print("   Welcome to the   ", 1);
            lcd_print("  SummitWand v1.0!  ", 2);
            lcd_print("    Main Menu    ", 3);
            PORTB |= (1 << PB0);
            break;
        case 1:
            // Display screen 2: Temperature Sensor
            lcd_clear(0);
            lcd_print("Temperature Data: ", 1);
            PORTB &= ~(1 << PB0);
            break;
        case 2:
            // Display screen 3: Heart rate sensor
            lcd_clear(0);
            lcd_print("Heart Rate: ",1);
            PORTB |= (1 << PB0);
            break;
        case 3:
            // Display screen 4: GPS Data
            lcd_clear(0);
            lcd_print("GPS Data: ",1);
            PORTB &= ~(1 << PB0);
            break;
        case 4:
            // Display screen 5: Accelerometer
            lcd_clear(0);
            lcd_print("Accelerometer: ",1);
            PORTB |= (1 << PB0);
            break;
        default:
            break;
    }
}

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

    // Set up push button
    DDRD &= ~(1 << DDD2); // Set PD2 as input (for push button)
    PORTD |= (1 << PD2); // Enable internal pull-up resistor for PD2

    // Enable pin change interrupt on PD2
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT18);

    // Enable global interrupts
    sei();

    // Main Program Loop
    while(1){
        if (buttonPressed) {
            screenCounter++;
            if (screenCounter >= 5) {
                screenCounter = 0; 
            }
            updateScreen();
            buttonPressed = 0;
        }
    };

    return 0;  
}
