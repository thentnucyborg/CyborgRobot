/*
  https://playground.arduino.cc/Interfacing/Python
  Using FastLED library:
  - Download the FastLED library: http://fastled.io/ as .zip.
  - In Arduino IDE: Sketch -> Include library -> Add .ZIP library
*/
#include <FastLED.h>
#define NUM_LEDS 240
#define DATA_PIN 6
#define BAUDRATE 115200

// BUFFER AND VARIABLES FOR RECEIVING A BYTEARRAY
uint8_t buffer[NUM_LEDS*3];   // Each led has 3 bytes of data (One for each color value)
int numBytesRead = 0;   // How many bytes have we read into the buffer
bool gotData = false;   // Got all data we needed to set leds

// INITIALIZE LEDS
CRGB leds[NUM_LEDS];

// SETTING LEDS FROM RECEIVED DATA
void setLedsFromBuffer(){
  int colorCount = 0;
  int ledCount = 0;
  for(int i = 0; i< NUM_LEDS*3; i++){
    switch(colorCount){
        case 0:
          leds[ledCount].r = buffer[i];
          break;
        case 1:
          leds[ledCount].g = buffer[i];
          break;
        case 2:
          leds[ledCount].b = buffer[i];
          break;
      }
      colorCount++;
      if(colorCount == 3){
        colorCount = 0;
        ledCount++;
      }
  }
}

// SETUP LEDS AND SERIAL COMMUINICATION
void setup(){
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  FastLED.clear();
  FastLED.show();
  Serial.begin(BAUDRATE, SERIAL_8N1);     //Starting serial communication, 8 data bits, no parity, 1 stop bit
}

void loop(){
  if(gotData){
    setLedsFromBuffer();
    FastLED.show();
    gotData = false;
  }
}

// SERIAL
/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.

  https://www.arduino.cc/en/Reference/SerialEvent
  https://www.arduino.cc/en/Tutorial/SerialEvent
*/

// Handle incoming serial data from PC, Pioneer LX or Raspberry Pi
void serialEvent(){
  while(Serial.available() && !gotData){
    buffer[numBytesRead] = Serial.read();
    numBytesRead++;
    if(numBytesRead == NUM_LEDS*3){
      gotData = true;
      numBytesRead = 0;
    }
  }
}
