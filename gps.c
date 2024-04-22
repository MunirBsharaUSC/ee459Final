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

void parse_gpgga(void) {
    char *token;
    uint8_t count = 0;

    char data_copy[128];
    strcpy(data_copy, gps_buffer);

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
}

char isGPSLocked(void) {
    if (strncmp(gps_buffer, "$GPGGA", 6) != 0) {
        return 0;
    }
    else return 1;

    // Find the comma after the 5th field (GPS Quality Indicator)
    const char* comma = strchr(gps_buffer, ',');
    int i;
    for (i = 0; i < 4; ++i) {
        comma = strchr(comma + 1, ',');
        if (comma == NULL) {
            return 0; 
        }
    }

    // Check if the GPS Quality Indicator is '1' (indicating a GPS fix)
    char quality_indicator = *(comma + 1);
    if (quality_indicator == '1' || quality_indicator == '2') {
        return 1;
    }

    return 0;
}
