#include "lcd.h"

void lcd_send_command(uint8_t command) {
    uint8_t control_byte = 0x80; // Control byte to indicate a command (adjust based on your LCD's protocol)
    uint8_t command_packet[2] = {control_byte, command};

    // Send the command using i2c_io; adjust the device_addr as necessary
    i2c_io(LCD_ADDR, command_packet, 2, NULL, 0);
}

void lcd_send_data(char *command) {
    uint8_t control_byte = 0x40; // Control byte to indicate a command (adjust based on your LCD's protocol)
    uint8_t command_packet[strlen(command)+1];
    command_packet[0] = control_byte;
    uint8_t i = strlen(command);

    while(i!=0){
        command_packet[strlen(command)-i+1]=command[strlen(command)-i];
        i--;
    }

    // Send the command using i2c_io; adjust the device_addr as necessar
    i2c_io(LCD_ADDR, command_packet, strlen(command)+1, NULL, 0);
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
    _delay_ms(5); // Wait for command to execute

    // Entry Mode Set: set the text entry mode (adjust as necessary)
    lcd_send_command(LCD_ENTRY_MODE_SET);
    _delay_us(120); // Wait for command to execute
}

// Function to directly print output strings to the LCD
void lcd_print(char* output, uint8_t row){
    char row_out[21];
    memset(row_out, ' ', 21);

    if(row >= 1 && row <=4){ // Print to a specific row (truncates to 20 characters)
        if (row == 1)
            lcd_send_command(LCD_MOVE_ROW1);
        else if(row == 2)
            lcd_send_command(LCD_MOVE_ROW2);
        else if (row == 3)
            lcd_send_command(LCD_MOVE_ROW3);
        else if (row == 4)
            lcd_send_command(LCD_MOVE_ROW4);
        _delay_us(50);

        uint8_t i;
        for(i=0; i<20; i++)
            row_out[i] = output[i];
        row_out[20] = '\0';
        lcd_send_data(row_out);
        _delay_us(50);
    }
    else{// Print to entire display (truncates to 80 characters with proper wrapping)
        uint8_t i = 0;
        
        // Print row 1
        lcd_send_command(LCD_MOVE_ROW1);
        for(i=0; i<20; i++)
            row_out[i] = output[i];
        row_out[20] = '\0';
        lcd_send_data(row_out);
        _delay_us(50);

        // Print row 2
        if(strlen(output) > 20){
            lcd_send_command(LCD_MOVE_ROW2);
            _delay_us(50);
            for(i=0; i<20; i++)
                row_out[i] = output[20+i];
            row_out[20] = '\0';
            lcd_send_data(row_out);
            _delay_us(50);
        }

        // Print row 3
        if(strlen(output) > 40){
            lcd_send_command(LCD_MOVE_ROW3);
            _delay_us(50);
            for(i=0; i<20; i++)
                row_out[i] = output[40+i];
            row_out[20] = '\0';
            lcd_send_data(row_out);
            _delay_us(50);
        }

        // Print row 4
        if(strlen(output) > 60){
            lcd_send_command(LCD_MOVE_ROW4);
            _delay_us(50);
            for(i=0; i<20; i++)
                row_out[i] = output[60+i];
            row_out[20] = '\0';
            lcd_send_data(row_out);
            _delay_us(50);
        }
    }
}

// Wrapper function to clear the LCD display
void lcd_clear(uint8_t row){
    if(row >= 1 && row <= 4) // Clear a specific row with space printing
        lcd_print("                    ", row);
    else{ // Hard clear entire display
        lcd_send_command(LCD_CLEAR_DISPLAY);
        _delay_ms(5);
    }
}
