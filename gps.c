#include "gps.h"

extern volatile uint8_t gps_data_ready;
extern volatile char gps_buffer[128];
extern char latitude[20];
extern char longitude[20]; 
extern char dir1[2];
extern char dir2[2];

ISR(USART_RX_vect) {
    char received_char = UDR0; 
    if (received_char == '\n') {
        gps_buffer[buffer_index] = '\0'; 
        buffer_index = 0; 
        gps_data_ready = 1; 
    } else {
        gps_buffer[buffer_index++] = received_char; 
        if (buffer_index > 127) {
            buffer_index = 0; 
        }
    }
}

void parse_gpgga(void) {

    char *token;
    uint8_t count = 0;

    unsigned char data_copy[strlen(gps_buffer) + 1];
    strcpy(data_copy, gps_buffer);
    data_copy[strlen(gps_buffer) + 1] = '\0';

    token = strtok(data_copy, ",");
    while (token != NULL) {
        count++;
        if (count == 2) {
            strcpy(latitude, token);
        }
        else if (count == 3){
            strcpy(dir1, token);
        }
        else if (count == 9){
            strcpy(dir2, token);
        }
        else if (count == 10){
            strcpy(longitude, token);
            break;
        }
        token = strtok(NULL, ",");
    }
}

void parse_gprmc(void){

    char *token;
    uint8_t count = 0;

    unsigned char data_copy[strlen(gps_buffer) + 1];
    strcpy(data_copy, gps_buffer);
    data_copy[strlen(gps_buffer) + 1] = '\0';

    token = strtok(data_copy, ",");
    while (token != NULL) {
        count++;
        if (count == 4) {
            strcpy(latitude, token);
        }
        else if (count == 5){
            strcpy(dir1, token);
        }
        else if (count == 6) {
            strcpy(longitude, token);
            break;
        }
        // else if (count == 7){
        //     strcpy(dir2, token);
        //     break;
        // }
        token = strtok(NULL, ",");
    }
}
