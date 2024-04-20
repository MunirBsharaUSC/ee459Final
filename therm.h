#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define THERM_PORT PORTC
#define THERM_DDR DDRC
#define THERM_PIN PINC
#define THERM_DQ PC3
#define THERM_DECIMAL_STEPS_12BIT 625
#define THERM_INPUT_MODE() THERM_DDR&=~(1<<THERM_DQ)
#define THERM_OUTPUT_MODE() THERM_DDR|=(1<<THERM_DQ)
#define THERM_LOW() THERM_PORT&=~(1<<THERM_DQ)
#define THERM_HIGH() THERM_PORT|=(1<<THERM_DQ)
#define THERM_CMD_SKIPROM 0xcc
#define THERM_CMD_CONVERTTEMP 0x44
#define THERM_CMD_RSCRATCHPAD 0xbe

extern uint8_t therm_reset(void);
extern void therm_write_bit(uint8_t);
extern uint8_t therm_read_byte(void);
extern uint8_t therm_read_bit(void);
extern void therm_write_byte(uint8_t);
extern void therm_read_temperature(char *);
