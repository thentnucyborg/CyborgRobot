//
// Written by Waloen for the NTNU Cyborg Spring 2017
// Code is for the Start Box
// See my thesis for more documentation
//
#include <util/delay.h>
#include <avr/io.h>
#include <string.h>
#include <stdlib.h>
#include "oled.h"
#include "spi.h"


void oled_init(void){
    spi_init();
    DDRD |= 0b00001100; // set reset and data to output

    // oled reset
    PORTD |= 0x08;
    _delay_ms(10);
    PORTD &= ~(0x08);
    _delay_ms(10);
    PORTD |= 0x08;
    _delay_ms(10);
    PORTD &= ~(0x08);
    _delay_ms(10);
    PORTD |= 0x08;
    _delay_ms(10);

//     START OF init sequence
//     Init sequence thanks to Adafruit's SSD1306 driver
//     Code is changed but the sequence is the same.
//     https://github.com/adafruit/Adafruit_SSD1306
    oled_command(0xAE);// Turn OFF
    oled_command(0xD5);// clockdivider command
    oled_command(0x80);// ratio
    oled_command(0xA8);// multiplex command
    oled_command(63);  // height
    oled_command(0xD3);// display offset command
    oled_command(0x00);// none
    oled_command(0x40);// startline 0
    oled_command(0x8D);// chargepump
    oled_command(0x14);
    oled_command(0x20);// memory mode
    oled_command(0x00);              
    oled_command(0xA1);
    oled_command(0xC8);
    oled_command(0xDA);// 0xDA
    oled_command(0x12);
    oled_command(0x81);// contrast command
    oled_command(0xCF);
    oled_command(0xd9);//set precharge
    oled_command(0xF1);
    oled_command(0xDB);
    oled_command(0x40);
    oled_command(0xA4);
    oled_command(0xA6);// Normal
    oled_command(0x2E);
    oled_command(0xAF);// turn ON
    // END OF init sequence
}

void oled_write_line(char* s, uint8_t line, uint8_t col){
    oled_command(0x21);  // Set column address
    oled_command(col);   // What column to start at
    oled_command(127);   //  end at 128 pixel in width

    oled_command(0x22);  // Set page address
    oled_command(line);  // What line to start on
    oled_command(7);     // end at line 8

    spi_deCS();
    PORTD |= (1<<PD2); // Start writing data
    spi_CS();
    int rest = 0;
    for(int i = 0; i < 128; i++){
        if(s[i] == '\0'){rest = 128-(i*8)-col; break;}
        for(int j = 0; j < 8; j++){
            spi_write(pgm_read_byte(&font8x8_data[s[i]-32][j]));
        }
    }
    spi_deCS();
}

void oled_clear(void){
    oled_command(0x21); // Set column address
    oled_command(0);   // What column to start at
    oled_command(127); // 128 pixel in width

    oled_command(0x22); // Set page address
    oled_command(0); // What line to start on
    oled_command(7); // end at line 8
  
    spi_deCS();
    PORTD |= (1<<PD2); // Start writing data
    spi_CS();
    
    for(int i = 0; i<128*8; i++){spi_write(0x00);} // clear oled
    spi_deCS();
}

void oled_command(uint8_t c){
    spi_deCS();
    PORTD &= ~(1<<PD2); // Start writing command
    spi_CS();
    spi_write(c); // Write command
    spi_deCS();
}
