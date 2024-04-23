#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>

#include "i2c.h"

#define ACCEL_ADDR_WRITE        0x3A    // I2C Address for write operation
#define ACCEL_ADDR_READ         0x3B    // I2C Address for read operation
#define ACCEL_POWER_CTRL_REG    0x2D    // Power control register for toggling standby mode
#define ACCEL_ENABLE            0x08    // Enable measure bit in POWER_CTRL reg
#define ACCEL_DATA_FORMAT_REG   0x31    // Data format register
#define ACCEL_DATA_FORMAT       0x0B    // Data format (Full 16 bit res with +/- 16g)
#define ACCEL_DATA_REG          0x32    // Data output register (X at 32-33, Y at 34-35, Z at 36-37) structured big-endian

extern void accel_init(void);
extern void accel_read(int16_t* x, int16_t* y, int16_t* z);
extern void pedometer(int16_t *,int16_t *, int16_t *, uint8_t *, uint8_t *, unsigned long *);

#define Z_THRESHOLD 150
#define DELAYTHRESHOLD 5
