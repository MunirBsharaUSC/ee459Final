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

    // Setup/Initialization
    USART_Init(MYUBRR);
    adc_init();
    accel_init();
    button_init();
    therm_reset();
    timer1_init();

    // Variables and Buffers
    char accel_buf[20];
    int16_t z = 0;
    timer_ticks = 0;
    
    uint8_t delayTime=0;
    uint8_t delayFlag=0;
    unsigned long step=0;
    unsigned long oldStep=0;

    unsigned long count=0;
    unsigned long beat_times[10]={0};
    uint8_t currIndex=0;
    uint8_t startIndex=0;
    uint8_t en=0;
    uint8_t prevSample=0;

  
    // Retrieve info from EEPROM
    state = eeprom_read_byte((void*) 0);
    state = (state >= 6 || state < 0) ? STATE_HOME : state; // Check state validity. Default to home if invalid

    // Display succesful boot
    _delay_ms(1000);
    lcd_clear(3);
    lcd_clear(4);
    lcd_print("   Boot Succesful!  ", 4);
    _delay_ms(1000);
    lcd_clear(0);
    sei();  // Enable interrupts after boto

    // Main Program Loop
    while(1){
        switch(state){
            case STATE_HOME:
                lcd_print(" |SUMMITWAND HOME|  ", 1);
                lcd_print("Press to toggle view", 2);
                lcd_print("1:Home 2:GPS 3:Pulse", 3);
                lcd_print("4:Temp 5:Acc 6:Trip ", 4);
            break;

            case STATE_GPS:
                lcd_print("     |GPS DATA|     ", 1);
            break;

            case STATE_PULSE:
                lcd_print("  |PULSE MONITOR|   ", 1);
                heartbeatCalc(accel_buf, &count, beat_times, &currIndex, &startIndex, &en, &prevSample);
                if(count==1){
                    lcd_clear(3);
                    lcd_print(accel_buf, 3);
                }
            break;

            case STATE_TEMP:
                lcd_print("Temp State", 0);
            break;

            case STATE_ACCEL:
                lcd_print("Accel State", 0);
                pedometer(&z, &delayTime, &delayFlag, &step);
                if(oldStep!=step){
                    snprintf(accel_buf, 20, "Y:%ld", step);
                    lcd_clear(4);
                    lcd_print(accel_buf, 4);
                    oldStep=step;
                }
            break;

            case STATE_TRIP:
                lcd_print("Trip State", 0);
    
            break;

            case STATE_TRIP_RESET:
                lcd_print("RESET TRIP DATA", 0);
                _delay_ms(3000);
                state = STATE_TRIP;
                state_change = 1;
                lcd_clear(0);
            break;
        }
        if(state_change){
            state_change = 0;
            eeprom_update_byte((void*) 0, state);
            lcd_clear(0);
            _delay_ms(50);
            PCICR |= (1 << PCIE2);
        }
        _delay_ms(10); //Fixed loop delay
    };

    return 0;   // Never reached
}
