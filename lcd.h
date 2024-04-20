#include <util/delay.h>
#include <string.h>

#include "i2c.h"

#define ADDR2 0x78
#define LCD_CLEAR_DISPLAY 0x01
#define LCD_RETURN_HOME 0x02
#define LCD_ENTRY_MODE_SET 0x06
#define LCD_DISPLAY_ON 0x0F // Display on, cursor off, blink off
#define LCD_FUNCTION_SET 0x38 // 4-bit mode, 2 lines, 5x8 dots

extern void lcd_send_command(uint8_t);
extern void lcd_send_data(char *);
extern void lcd_init(void);
