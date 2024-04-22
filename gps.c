#include "gps.h"

extern volatile uint8_t gps_data_ready;
extern volatile char gps_buffer[128];
extern char latitude[20];
extern char longitude[20]; 

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

char parse_gpgga(void) {

    char *token;
    uint8_t count = 0;

    if (strncmp(gps_buffer,"$GPGGA", 6) == 0 ){ // GPGGA Data 

        char data_copy[strlen(gps_buffer) + 1];
        strcpy(data_copy, gps_buffer);
        data_copy[strlen(gps_buffer) -1] = '\0'; 

        token = strtok(data_copy, ",");
        while (token != NULL) {
            count++;
            if (count == 3) {
                strcpy(latitude, token);
            }
            else if (count == 5) {
                strcpy(longitude, token);
                break;
            }
            token = strtok(NULL, ",");
        }
        return 1;
    }
    else{ // Not GPGGA data
        return 0;
    }    
}

char isGPSLocked(void) {
    if (strncmp(gps_buffer, "$GPGGA", 6) == 0) {
        return 1;
    }
    else{
        return 0;
    }

    // // Find the comma after the 5th field (GPS Quality Indicator)
    // const char* comma = strchr(gps_buffer, ',');
    // int i;
    // for (i = 0; i < 4; ++i) {
    //     comma = strchr(comma + 1, ',');
    //     if (comma == NULL) {
    //         return 0; 
    //     }
    // }

    // // Check if the GPS Quality Indicator is '1' (indicating a GPS fix)
    // char quality_indicator = *(comma + 1);
    // if (quality_indicator == '1' || quality_indicator == '2') {
    //     return 1;
    // }

    return 0;
}
