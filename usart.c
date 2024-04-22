#include "usart.h"

void USART_Init(unsigned int ubrr){
    UBRR0 = ubrr;               // Set baud rate
    UCSR0C = (3 << UCSZ00);     // Set for asynchronous operation, no parity, one stop bit, 8 data bits
    UCSR0B |= (1 << TXEN0);     // Turn on transmitter
    UCSR0B |= (1 << RXEN0);     // Turn on receiver
    UCSR0B |= (1 << RXCIE0);    // Enable receiver interrupts
    DDRD |= (1 << PD1);
}

void USART_Transmit(unsigned char data){
    // Wait for empty transmit buffer
    while ( !( UCSR0A & (1<<UDRE0)) );
    // Put data into buffer, sends the data
    UDR0 = data;
}

unsigned char USART_Receive(void){
    // Wait for data to be received
    while ( !(UCSR0A & (1<<RXC0)) );
    // Get and return received data from buffer
    return UDR0;
}
