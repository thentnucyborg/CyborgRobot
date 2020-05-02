//
// Written by Waloen for the NTNU Cyborg Spring 2017
// Code is for the Start Box
// See my thesis for more documentation
//
#include "cyborg_menu.h"


int menu(){
    uint8_t selected = 0;

    while(1){
        oled_write_line(">", selected, 0);
        if(!(PINC & (1<<4))){	// Button down
            if(selected != 7){
                oled_write_line(" ", selected, 0);
                selected++;
                oled_write_line(">", selected, 0);
            }
        }else if(!(PINC & (1<<3))){  // Button select
            return selected;
        }else if(!(PINC & (1<<5))){  // Button up
            if(selected != 0){
                oled_write_line(" ", selected, 0);
                selected--;
                oled_write_line(">", selected, 0);
            }
        }
        _delay_ms(150); // time between button read
    }
}


    // EDIT THIS FOR NEW SEQUENCE NAMES
    // oled_write_line("TEXT", line, start_page)
    // lines in range 0-7
    // start_page must start at 8 or later because of selection arrow

void present_modes(uint8_t selected){
    oled_write_line("Normal start", 0, 8);
    oled_write_line("Demo Aria", 1, 8);
    oled_write_line("ARNL", 2, 8);
    oled_write_line("Sequence 4", 3, 8);
    oled_write_line("Sequence 5", 4, 8);
    oled_write_line("Sequence 6", 5, 8);
    oled_write_line("Sequence 7", 6, 8);
    oled_write_line("Sequence 8", 7, 8);
    if (selected<9){
        oled_write_line("X", selected, 120);
    }
}

//EDIT THIS FOR NEW SEQUENCE NAMES AND PROPER TERMINATION
void present_current_mode(uint8_t selected){
    char *mode = "None";
    switch(selected){
        case 0:
           mode = "Normal start";
            break;
        case 1:
           mode = "Demo Aria";
            break;
        case 2:
            mode = "ARNL"; 
        default:
            break;
    }
    oled_write_line("Current mode:", 0, 8);
    oled_write_line(mode, 1 , 8);
    oled_write_line("Shut down current mode.", 3, 8);
}
