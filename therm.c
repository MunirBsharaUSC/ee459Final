#include "therm.h"

inline __attribute__((gnu_inline)) void therm_delay(uint16_t delay){ while(delay--) asm volatile("nop"); }

uint8_t therm_reset(){
    uint8_t i;
    THERM_LOW();
    THERM_OUTPUT_MODE();
    _delay_us(480);
    //therm_delay(us(480));

    THERM_INPUT_MODE();
    _delay_us(60);
    //therm_delay(us(60));

    i=(THERM_PIN & (1<<THERM_DQ));
    _delay_us(420);
    //therm_delay(us(420));

    return i;
}

void therm_write_bit(uint8_t bit){

    THERM_LOW();
    THERM_OUTPUT_MODE();
    _delay_us(1);
    //therm_delay(us(1));

    if(bit) THERM_INPUT_MODE();

    _delay_us(60);
    //therm_delay(us(60));
    THERM_INPUT_MODE();
}

uint8_t therm_read_bit(void){
    uint8_t bit=0;

    THERM_LOW();
    THERM_OUTPUT_MODE();
    _delay_us(1);

    THERM_INPUT_MODE();
    _delay_us(14);
    //therm_delay(us(14));

    if(THERM_PIN&(1<<THERM_DQ)) bit=1;

    _delay_us(45);
    //therm_delay(us(45));
    return bit;
}

uint8_t therm_read_byte(void){
    uint8_t i=8, n=0;

    while(i--){
        n>>=1;
        n|=(therm_read_bit()<<7);
    }

    return n;
}

void therm_write_byte(uint8_t byte){
    uint8_t i=8;

    while(i--){
        therm_write_bit(byte&1);
        byte>>=1;
    }
}

void therm_read_temperature(char *buffer){
    uint8_t temperature[2];
    int8_t digit;
    uint16_t decimal;

    therm_reset();
    therm_write_byte(THERM_CMD_SKIPROM);
    therm_write_byte(THERM_CMD_CONVERTTEMP);

    while(!therm_read_bit());

    therm_reset();
    therm_write_byte(THERM_CMD_SKIPROM);
    therm_write_byte(THERM_CMD_RSCRATCHPAD);

    temperature[0]=therm_read_byte();
    temperature[1]=therm_read_byte();
    therm_reset();

    digit=temperature[0]>>4;
    digit|=(temperature[1]&0x7)<<4;

    decimal=temperature[0]&0xf;
    decimal*=THERM_DECIMAL_STEPS_12BIT;

    if(digit>=40){
        sprintf(buffer, "    %+d.%04u  HOT     ", digit, decimal);
    }
    else if(digit>=30){
        sprintf(buffer, "    %+d.%04u  WARM    ", digit, decimal);
    }
    else if(digit>=20){
        sprintf(buffer, "    %+d.%04u  ROOM    ", digit, decimal);
    }
    else{
        sprintf(buffer, "    %+d.%04u  COLD    ", digit, decimal);
    }

}
