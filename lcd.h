#include <util/delay.h>
#include <string.h>

#include "i2c.h"

#define LCD_ADDR            0x78
#define LCD_CLEAR_DISPLAY   0x01
#define LCD_RETURN_HOME     0x02
#define LCD_ENTRY_MODE_SET  0x06
#define LCD_DISPLAY_ON      0x0C    // Display on, cursor off, blink off
#define LCD_FUNCTION_SET    0x38    // 4-bit I2C mode, 1 lines, 5x8 dots
#define LCD_MOVE_ROW1       0x80    // Move cursor to start of row 1
#define LCD_MOVE_ROW2       0xC0    // Move cursor to start of row 2
#define LCD_MOVE_ROW3       0x94    // Move cursor to start of row 3
#define LCD_MOVE_ROW4       0xD4    // Move cursor to start of row 4

extern void lcd_send_command(uint8_t);
extern void lcd_send_data(char *);
extern void lcd_init(void);
extern void lcd_print(char* output, uint8_t row);
extern void lcd_clear(uint8_t row);
