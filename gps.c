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

bool isGPSLocked(const char *buffer[], int *length) {
    // Check if the buffer starts with "$GPGGA"
    if (length < 10 || string(buffer, 6) != "$GPGGA") {
        return false;
    }

    // Remove the leading "$"
    string data(buffer + 1, length - 1);
    
    // Split the sentence into fields
    stringstream ss(data);
    vector<string> fields;
    string field;
    while (getline(ss, field, ',')) {
        fields.push_back(field);
    }

    // Check if the number of fields is correct and if the GPS quality indicator is 1
    if (fields.size() >= 7 && fields[6] == "1") {
        return true;
    }

    return false;
}