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
    int16_t x;
    int16_t y;
    int16_t z;
    timer_ticks = 0;
    
    memset(latitude, ' ', 20);
    memset(longitude, ' ', 20);
    gps_data_ready = 0;
    
    uint8_t delayTime=0;
    uint8_t delayFlag=0;
    uint8_t firstEntry=1;
    unsigned long step=0;
    unsigned long oldStep=0;

    unsigned long count=0;
    unsigned long beat_times[10]={0};
    uint8_t currIndex=0;
    uint8_t startIndex=0;
    uint8_t en=0;
    uint8_t prevSample=0;

    uint8_t counter=0;


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
    sei();  // Enable interrupts after boot

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
                while(!gps_data_ready);
                if(isGPSLocked()){
                    lcd_print(" GPS Lock Obtained! ", 2);  
                    parse_gpgga();
                    char out_buf[20];
                    sprintf(out_buf, "LAT : %s", latitude);
                    lcd_print(out_buf, 3);
                    sprintf(out_buf, "LONG: %s", longitude);
                    lcd_print(out_buf, 4);   
                    gps_data_ready = 0;
                }
                else{ // No GPS Lock
                    lcd_print("    No GPS Lock     ", 2);
                    lcd_print("    Attempting      ", 3);
                    lcd_print("    Connection...   ", 4);
                }
            break;

            case STATE_PULSE:
                lcd_print("  |PULSE MONITOR|   ", 1);
                heartbeatCalc(accel_buf, &count, beat_times, &currIndex, &startIndex, &en, &prevSample);

                if(count==1){
                    lcd_clear(3);
                    lcd_print(accel_buf, 3);
                }
                if(firstEntry){
                    lcd_clear(3);
                    lcd_print("      NO BEATS      ", 3);
                    firstEntry=0;
                }
            break;

            case STATE_TEMP:
                lcd_print("   |THERMOMETER|    ", 1);
                lcd_print("      Celsius      ", 2);
                therm_read_temperature(accel_buf);
                lcd_clear(3);
                lcd_print(accel_buf, 3);
                firstEntry=1;

            break;

            case STATE_ACCEL:
                lcd_print("  |ACCELEROMETER|   ", 1);
                pedometer(&x, &y, &z, &delayTime, &delayFlag, &step);

                if(counter++==9){
                    counter=0;
                    snprintf(accel_buf, 20, "X:%+4d Y:%+4d Z:%+4d", x,y,z);
                    lcd_clear(2);
                    lcd_print(accel_buf, 2);
                }


                if(oldStep!=step){
                    snprintf(accel_buf, 20, "   Stick Steps:%ld", step);
                    lcd_clear(3);
                    lcd_print(accel_buf, 3);
                    snprintf(accel_buf, 20, "   Est. Steps:%ld", (step*2));
                    lcd_clear(4);
                    lcd_print(accel_buf, 4);
                    oldStep=step;
                }
                if(firstEntry){
                    snprintf(accel_buf, 20, "   Stick Steps:%ld", step);
                    lcd_clear(3);
                    lcd_print(accel_buf, 3);
                    snprintf(accel_buf, 20, "   Est. Steps:%ld", (step*2));
                    lcd_clear(4);
                    lcd_print(accel_buf, 4);
                    oldStep=step;
                    firstEntry=0;
                }
            break;

            case STATE_TRIP:
                lcd_print("    |TRIP DATA|     ", 1);
                lcd_print("  Hold 3s to reset  ", 2);
                lcd_print("STEPS: ", 3);
                lcd_print("TIME: ", 4);
                firstEntry=1;

            break;

            case STATE_TRIP_RESET:
                lcd_clear(0);
                lcd_print(" |RESET TRIP DATA|  ", 1);
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
            enable_button_interrupt();
        }
        _delay_ms(10); //Fixed loop delay
    };

    return 0;   // Never reached
}
