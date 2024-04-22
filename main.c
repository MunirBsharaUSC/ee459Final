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
    char output_buf[20];
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
    timer_ticks = 0;

    memset(latitude, ' ', 20);
    memset(longitude, ' ', 20);
    //memset(dir1, ' ', 5);
    //memset(dir2, ' ', 5);
    gps_data_ready = 0;

    uint8_t delayTime = 0;
    uint8_t delayFlag = 0;
    uint8_t firstEntry = 1;
    unsigned long trip_step = 0;
    unsigned long old_trip_step = 0;


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
    trip_time = eeprom_read_dword((void*) 4);
    trip_step = eeprom_read_dword((void*) 8);

    // Display succesful boot
    _delay_ms(1000);
    lcd_clear(3);
    lcd_clear(4);
    lcd_print("   Boot Succesful!  ", 4);
    _delay_ms(1000);
    lcd_clear(0);
    sei();  // Enable interrupts after boot
    timer2_init(); // Start clock

    // Main Program Loop
    while(1){
        switch(state){
            case STATE_HOME:
                lcd_print(" |SUMMITWAND HOME|  ", 1);
                lcd_print("Press to toggle view", 2);
                lcd_print("1:Home 2:GPS 3:Pulse", 3);
                lcd_print("4:Temp 5:Acc 6:Trip ", 4);
                firstEntry=1;
            break;

            case STATE_GPS:
                lcd_print("     |GPS DATA|     ", 1);
                firstEntry=1;
                while(!gps_data_ready);

                // lcd_print(gps_buffer, 0);

                // if (strncmp(gps_buffer, "$GPGGA", 6) == 0){
                //     parse_gpgga();
                //     lcd_print("    Locked GPGGA   ", 2);
                //     sprintf(output_buf, "LAT : %s %s", latitude, dir1);
                //     lcd_print(output_buf, 3);
                //     sprintf(output_buf, "LONG: %s %s", longitude, dir2);
                //     lcd_print(output_buf, 4);
                //     gps_data_ready = 0;
                // }

                if (strncmp(gps_buffer, "$GPRMC", 6) == 0){
                    lcd_print("    Locked GPRMC   ", 2);
                    parse_gprmc();
                    sprintf(output_buf, "LAT : %s %s", latitude, dir1);
                    lcd_print(output_buf, 3);
                    sprintf(output_buf, "LONG: %s %s", longitude, "W");
                    lcd_print(output_buf, 4);
                    gps_data_ready = 0;
                }
                
            break;

            case STATE_PULSE:
                lcd_print("  |PULSE MONITOR|   ", 1);
                heartbeatCalc(output_buf, &count, beat_times, &currIndex, &startIndex, &en, &prevSample);
                if(count==1){
                    lcd_clear(3);
                    lcd_print(output_buf, 3);
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
                therm_read_temperature(output_buf);
                lcd_clear(3);
                lcd_print(output_buf, 3);
                firstEntry=1;

            break;

            case STATE_ACCEL:
                lcd_print("  |ACCELEROMETER|   ", 1);
                if(counter++==9){
                    counter=0;
                    snprintf(output_buf, 20, "X:%+4d Y:%+4d Z:%+4d", x,y,z);
                    lcd_clear(2);
                    lcd_print(output_buf, 2);
                }
                if(old_trip_step !=trip_step){
                    snprintf(output_buf, 20, "  Stick Steps:%lu", trip_step);
                    lcd_clear(3);
                    lcd_print(output_buf, 3);
                    snprintf(output_buf, 20, "   Est. Steps:%lu", (trip_step*2));
                    lcd_clear(4);
                    lcd_print(output_buf, 4);
                    firstEntry=0;
                }
               else if(firstEntry){
                    snprintf(output_buf, 20, "  Stick Steps:%lu", trip_step);
                    lcd_clear(3);
                    lcd_print(output_buf, 3);
                    snprintf(output_buf, 20, "   Est. Steps:%lu", (trip_step*2));
                    lcd_clear(4);
                    lcd_print(output_buf, 4);
                    firstEntry=0;
                }
            break;

            case STATE_TRIP:
                lcd_print("    |TRIP DATA|     ", 1);
                lcd_print("  Hold 3s to reset  ", 2);
                snprintf(output_buf, 20, "STEPS: %lu", trip_step*2);
                lcd_print(output_buf, 3);
                snprintf(output_buf, 20, " TIME: %02lu:%02lu:%02lu", (trip_time/3600), (trip_time/60), (trip_time%60));
                lcd_print(output_buf, 4);
                firstEntry=1;
            break;

            case STATE_TRIP_RESET:
                lcd_clear(0);
                state = STATE_TRIP;
                state_change = 1;
                trip_step=0;
                trip_time = 0;
                lcd_print(" |RESET TRIP DATA|  ", 1);
                _delay_ms(1000);
                lcd_print("    Returning to    ", 3);
                lcd_print("    trip view...    ", 4);
                _delay_ms(2000);
                lcd_clear(0);
            break;
        }
        if(state_change){
            state_change = 0;
            eeprom_update_byte((void*) 0, state);
            lcd_clear(0);
            _delay_ms(100);
            enable_button_interrupt();

        }
        if(old_trip_step !=trip_step){
            eeprom_update_dword((void*) 8, trip_step);
            old_trip_step = trip_step;
        }
        if(timer_update){
            eeprom_update_dword((void*) 4, trip_time);
            timer_update = 0;
        }
        pedometer(&x, &y, &z, &delayTime, &delayFlag, &trip_step);
        _delay_ms(10); //Fixed loop delay
    };

    return 0;   // Never reached
}
