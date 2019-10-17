/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the Uno and
  Leonardo, it is attached to digital pin 13. If you're unsure what
  pin the on-board LED is connected to on your Arduino model, check
  the documentation at http://www.arduino.cc

  This example code is in the public domain.

  modified 8 May 2014
  by Scott Fitzgerald
 */

#include "Queue.h"

Queue<uint8_t*> LEDqueue(220);
uint8_t* singleLEDrow;

int incomingByte = 0;
unsigned long nrBytes = 0;
String inputString = "";
String stateString = "None";
bool stringComplete = false;


char rawData[255];
void setup(){
// Open serial connection.
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    delay(250);              
    digitalWrite(13, LOW);    
    delay(250);  
    Serial.begin(250000);
 
}



void serialEvent(){
  while(Serial.available()){
    nrBytes++;
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
//    if(inChar == '|'){
//      String reply = String(nrBytes);
//      Serial.println(reply);
//    }

  }

  
}


void stringToIntArray(String string){
  uint8_t stuff[240];
  for(int i = 0; i < string.length() - 1; i++){
    stuff[i] = uint8_t(string[i]);
//    Serial.println(LEDrow[i]);
    Serial.println(stuff[i]);
  }
  delay(100);
  LEDqueue.push(stuff);
}
 
void loop(){
  if(stringComplete){
    
    Serial.println(inputString);
    Serial.println("Length of inputString = " + inputString.length());
    delay(10);
    stringToIntArray(inputString);
    inputString = "";
    stringComplete = false; 
  }
//  uint8_t* newArray;
//  newArray = LEDqueue.pop();
//  for(int i = 0; i < 60; i++){
//    Serial.println(newArray[i]);
//  }
  delay(10);

}

