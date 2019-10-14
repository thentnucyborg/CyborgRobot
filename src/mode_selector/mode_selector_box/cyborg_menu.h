//
// Written by Waloen for the NTNU Cyborg Spring 2017
// Code is for the Start Box
// See my thesis for more documentation
//
#ifndef CYBORG_MENU_H_INCLUDED
#define CYBORG_MENU_H_INCLUDED

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include "spi.h"
#include "USART.h"
#include "oled.h"

int menu();
void present_modes(uint8_t chosen_seq);
void present_current_mode(uint8_t chosen_seq);

#endif // USART_H_INCLUDED
