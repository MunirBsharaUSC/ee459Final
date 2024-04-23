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

// void parse_gpgga(void) {

//     char *token;
//     uint8_t gps_count = 0;

//     char data_copy[strlen(gps_buffer) + 1];
//     strcpy(data_copy, gps_buffer);
//     data_copy[strlen(gps_buffer) + 1] = '\0';

//     token = strtok(data_copy, ",");
//     while (token != NULL) {
//         gps_count++;
//         if (gps_count == 2) {
//             strcpy(latitude, token);
//         }
//         else if (gps_count == 3){
//             strcpy(dir1, token);
//         }
//         else if (gps_count == 9){
//             strcpy(dir2, token);
//         }
//         else if (gps_count == 10){
//             strcpy(longitude, token);
//             break;
//         }
//         token = strtok(NULL, ",");
//     }
// }

void parse_gprmc(void){

    char* token;
    uint8_t gps_count = 0;

    char data_copy[strlen((const char*)gps_buffer) + 1];
    strcpy(data_copy, (const char*)gps_buffer);
    data_copy[strlen((const char*)gps_buffer) - 1] = '\0';

    token = strtok(data_copy, ",");
    while (token != NULL) {
        gps_count++;
        if (gps_count == 4) {
            strcpy(latitude, token);
        }
        else if (gps_count == 5){
            strcpy(dir1, token);
        }
        else if (gps_count == 6) {
            strcpy(longitude, token);
            break;
        }
        // else if (gps_count == 7){
        //     strcpy(dir2, token);
        //     break;
        // }
        token = strtok(NULL, ",");
    }

    // Fix longitude and latitude decimal position (unverified)
    uint8_t i = 0;
    for(i=0; i<strlen((const char*) latitude); i++){
        if(latitude[i] == '.'){
            latitude[i] = latitude[i-1];
            latitude[i-1] = latitude[i-2];
            latitude[i-2] = '.';
            break;
        }
    }
    for(i=0; i<strlen((const char*) longitude); i++){
        if(longitude[i] == '.'){
            longitude[i] = longitude[i-1];
            longitude[i-1] = longitude[i-2];
            longitude[i-2] = '.';
            break;
        }
    }
}
