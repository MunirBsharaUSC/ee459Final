#include "lcd.h"

void lcd_send_command(uint8_t command) {
    uint8_t control_byte = 0x80; // Control byte to indicate a command (adjust based on your LCD's protocol)
    uint8_t command_packet[2] = {control_byte, command};

    // Send the command using i2c_io; adjust the device_addr as necessary
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
