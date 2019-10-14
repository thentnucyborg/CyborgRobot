//
// Written by Waloen for the NTNU Cyborg Spring 2017
// Code is for the Start Box
// See my thesis for more documentation
//
#include <avr/io.h>
#include "spi.h"

void spi_init(void){
    DDRB |= (1<<DDB2) | (1<<DDB3) | (1<<DDB5); // Data direction
    SPCR |= (1<<SPE) | (1<<SPR0) | (1<<MSTR); //SPI master mode etc
}

void spi_write(uint8_t c){
    SPDR = c;
    while(!(SPSR & (1<<SPIF))){}
}

void spi_CS(){
    PORTB &= ~(1<<PB2);
}

void spi_deCS(){
    PORTB |= (1<<PB2);
}
