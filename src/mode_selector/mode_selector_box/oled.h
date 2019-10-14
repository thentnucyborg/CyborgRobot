//
// Written by Waloen for the NTNU Cyborg Spring 2017
// Code is for the Start Box
// See my thesis for more documentation
//
#ifndef OLED_H_INCLUDED
#define OLED_H_INCLUDED
#include <stdio.h>
#include <avr/pgmspace.h>
#include "font8x8.h"

void oled_init(void);
void oled_clear(void);
void oled_write_line(char* s, uint8_t line, uint8_t col);
void oled_command(uint8_t c);

#endif // OLED_H_INCLUDED
