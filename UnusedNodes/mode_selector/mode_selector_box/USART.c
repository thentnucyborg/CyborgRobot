//
// Written by Waloen for the NTNU Cyborg Spring 2017
// Code is for the Start Box
// See my thesis for more documentation
//
#include <avr/io.h>
#include <stdio.h>
#include "USART.h"

void USART_init(unsigned int ubrr){
    DDRD = (1<<PD1);
    UBRR0H = (unsigned char)(ubrr>>8); // Baudrate
    UBRR0L = (unsigned char)ubrr; // Baudrate
    UCSR0B = (1<<RXEN0)|(1<<TXEN0); // enable RX and TX
    UCSR0C = (1<<USBS0)|(3<<UCSZ00); // 8 data, 2 stop bit
    fdevopen(&USART_transmit,  &USART_receive); // printf to usart
}

void USART_transmit( unsigned char data ){
    while( !( UCSR0A & (1<<UDRE0)) ); //wait for ready
    UDR0 = data;
}

uint8_t USART_receive(){
    while( !(UCSR0A & (1<<RXC0)) ){} //wait for receive
    return UDR0;
}
