//
// Written by Waloen for the NTNU Cyborg Spring 2017
// Code is for the Start Box
// See my thesis for more documentation
//
#ifndef SPI_H_INCLUDED
#define SPI_H_INCLUDED
#include <stdio.h>

void spi_init(void);
void spi_CS();
void spi_deCS();
void spi_write(uint8_t c);

#endif // SPI_H_INCLUDED
