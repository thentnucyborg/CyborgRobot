//
// Written by Waloen for the NTNU Cyborg Spring 2017
// Modified by Areg for the NTNNU Cytborg Spring 2019
// Code is for the Start Box
// See my thesis for more documentation
//

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include "spi.h"
#include "USART.h"
#include "oled.h"
#include "cyborg_menu.h"

// Define speed and baudrate
#define F_CPU 16000000
#define FOSC 16000000
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1
typedef enum {false, true} bool;

int main(){
    // Init
    USART_init(MYUBRR);
    oled_init();
    oled_clear();
    oled_write_line("Waiting for Cyborg..", 0, 8);

    uint8_t chosen_seq = 9;
    bool change_requested = false;
    
    // Internal pullup on buttons
    DDRC |= 0b00111000;
    PORTC |= 0b00111000;

    // Wait for Cyborg to wake up
    char inc_message = 'm';
    while(inc_message != 'C'){
        inc_message = fgetc(stdin);
    }

    oled_write_line("Cyborg is awake", 3, 8);
    _delay_ms(1000);

    oled_clear();

    while(1){
        // present menu with available modes
        present_modes(chosen_seq);

        // Clear all but chosen sequence
        chosen_seq = menu();
        for(int i = 0; i<8; i++){
            if(i != chosen_seq){
                oled_write_line("                ", i, 0); // Write empty lines
            }         
        }
        // Send sequence number to the start scrip
        printf("Executing Sequence %i\n", chosen_seq+1);
        //_delay_ms(3000);
        while(inc_message != 'F'){
            inc_message = fgetc(stdin);
        }

        oled_clear();

        // present selected mode and option to change
        present_current_mode(chosen_seq);
        while (change_requested == false){
        chosen_seq = menu();
            if (chosen_seq ==3){
                change_requested = true;
                // Send sequence number
                printf("Executing shutdown %i\n", 3);
            }
        }
        oled_clear();
        oled_write_line("Shutting down current mode",1,8);

        while(inc_message != 'C'){
            inc_message = fgetc(stdin);
        } 
        // wait for confirmation that shutdown is complete
        oled_clear();
        change_requested = false;
        chosen_seq = 9;
        _delay_ms(200); // remove after testing
    }
}