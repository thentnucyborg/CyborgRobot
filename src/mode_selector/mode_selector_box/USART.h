//
// Written by Waloen for the NTNU Cyborg Spring 2017
// Code is for the Start Box
// See my thesis for more documentation
//
#ifndef USART_H_INCLUDED
#define USART_H_INCLUDED

#include <stdio.h>

void USART_init(unsigned int ubrr);
void USART_transmit(unsigned char data);
uint8_t USART_receive();

#endif // USART_H_INCLUDED
