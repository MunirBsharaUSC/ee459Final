#include "gps.h"

ISR(USART_RX_vect) {
    char received_char = UDR0; 
    if (received_char == '\n') {
        buffer[buffer_index] = '\0'; 
        buffer_index = 0; 
        data_ready = 1; 
    } else {
        buffer[buffer_index++] = received_char; 
        if (buffer_index >= MAX_BUFFER_SIZE - 1) {
            buffer_index = 0; 
        }
    }
}

void parse_gpgga(const char *data, char *latitude, char *longitude) {
    char gpsBuffer[20];
    char gpsBuffer1[20];
    char *token;
    int count = 0;

    char data_copy[strlen(data) + 1];
    strcpy(data_copy, data);

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
    // sprintf(gpsBuffer, "Latitude %s\n", latitude);
    // lcd_print(gpsBuffer, 1);
    
    // sprintf(gpsBuffer1, "Longitude %s", longitude);
    // lcd_print(gpsBuffer1, 2);
}

int isGPSLocked(const char buffer[], int length) {
    if (strncmp(buffer, "$GPGGA", 6) != 0) {
        return 0;
    }

    // Find the comma after the 7th field (GPS Quality Indicator)
    const char* comma = strchr(buffer, ',');
    for (int i = 0; i < 4; ++i) {
        comma = strchr(comma + 1, ',');
        if (comma == NULL) {
            return 0; 
        }
    }

    // Extract the GPS Quality Indicator
    char quality_indicator = *(comma + 1);

    // Check if the GPS Quality Indicator is '1' (indicating a GPS fix)
    if (quality_indicator == '1' || quality_indicator == '2') {
        return 1;
    }

    return 0;
}
