#include "accel.h"

void accel_init(void){

    uint8_t wdata[2];
    wdata[0] = ACCEL_POWER_CTRL_REG;
    wdata[1] = ACCEL_ENABLE;

    i2c_io(ACCEL_ADDR_WRITE, wdata, 2, NULL, 0);
    _delay_ms(5);
    
    wdata[0] = ACCEL_DATA_FORMAT_REG;
    wdata[1] = 0x0B;
    i2c_io(ACCEL_ADDR_WRITE, wdata, 2, NULL, 0);
    _delay_ms(5);
}

void accel_read(int16_t* x, int16_t* y, int16_t* z){
    uint8_t wdata[1];
    uint8_t rdata[6];
    wdata[0] = ACCEL_DATA_REG;
    
    i2c_io(ACCEL_ADDR_READ, wdata, 1, rdata, 6);
    _delay_ms(5);
    *x = ((rdata[1] << 8) | rdata[0]);
    *y = ((rdata[3] << 8) | rdata[2]);
    *z = ((rdata[5] << 8) | rdata[4]);
}
