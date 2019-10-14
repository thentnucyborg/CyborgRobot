/*
  https://playground.arduino.cc/Interfacing/Python
  Using FastLED library:
  - Download the FastLED library: http://fastled.io/ as .zip.
  - In Arduino IDE: Sketch -> Include library -> Add .ZIP library
*/


#include <FastLED.h>
// ONLY change these values
#define NUM_LEDS 6
#define DATA_PIN 6
#define BUTTON_PIN 2
#define MAX_BRIGHTNESS 100
#define BAUDRATE 1000000

// BUFFER AND VARIABLES FOR RECEIVING A BYTEARRAY
byte LED_buffer[NUM_LEDS*3];      // Each led has 3 bytes of data (One for each color value)
int numBytesRead = 0;             // How many bytes have we read into the buffer
bool gotData = false;             // Got all data we needed to set leds?

// INITIALIZE LEDS
CRGB leds[NUM_LEDS];

// SETTING LEDS FROM RECEIVED DATA
void setLedsFromBuffer(){
  int colorCount = 0;
  int ledCount = 0;
  for(int i = 0; i< NUM_LEDS*3; i++){
    switch(colorCount){
        case 0:
          leds[ledCount].r = LED_buffer[i];
          break;
        case 1:
          leds[ledCount].g = LED_buffer[i];
          break;
        case 2:
          leds[ledCount].b = LED_buffer[i];
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
  FastLED.setBrightness( MAX_BRIGHTNESS );
  Serial.begin(BAUDRATE, SERIAL_8N1);     //Starting serial communication, 8 data bits, no parity, 1 stop bit
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void CylonBounce(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay){

  for(int i = 0; i < NUM_LEDS-EyeSize-2; i++) {
    setAll(0,0,0);
    setPixel(i, red/10, green/10, blue/10);
    for(int j = 1; j <= EyeSize; j++) {
      setPixel(i+j, red, green, blue); 
    }
    setPixel(i+EyeSize+1, red/10, green/10, blue/10);
    showStrip();
    delay(SpeedDelay);
  }

  delay(ReturnDelay);

  for(int i = NUM_LEDS-EyeSize-2; i > 0; i--) {
    setAll(0,0,0);
    setPixel(i, red/10, green/10, blue/10);
    for(int j = 1; j <= EyeSize; j++) {
      setPixel(i+j, red, green, blue); 
    }
    setPixel(i+EyeSize+1, red/10, green/10, blue/10);
    showStrip();
    delay(SpeedDelay);
  }
  
  delay(ReturnDelay);
}

void colorWipe(byte red, byte green, byte blue, int SpeedDelay) {
  for(uint16_t i=0; i<NUM_LEDS; i++) {
      setPixel(i, red, green, blue);
      showStrip();
      delay(SpeedDelay);
  }
}

void loop(){
  if(gotData){
    setLedsFromBuffer();
    FastLED.show();     //30 us per pixel
    //Serial.flush();
    gotData = false;
  }
  if(digitalRead(BUTTON_PIN)==LOW){
    Serial.end();
    //HalloweenEyes(0xff, 0x00, 0x00, 1,1, true, 10, 100, 3000);
    //CylonBounce(0xff, 0, 0, 1, 10, 1000);
    for(uint8_t i= 0; i< 10; i++){
    colorWipe(0x00,0xff,0x00, 50);
    colorWipe(0x00,0x00,0x00, 50);
    }
  FastLED.clear();
  FastLED.show();
  Serial.begin(BAUDRATE, SERIAL_8N1);
  numBytesRead = 0;  
  }

  //delay(1);
}

void setPixel(int Pixel, byte red, byte green, byte blue) {
   // FastLED
   leds[Pixel].r = red;
   leds[Pixel].g = green;
   leds[Pixel].b = blue;
}

void showStrip() {
   FastLED.show();
}

void setAll(byte red, byte green, byte blue) {
  for(int i = 0; i < NUM_LEDS; i++ ) {
    setPixel(i, red, green, blue); 
  }
  showStrip();
}

// SERIAL
/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
  https://www.arduino.cc/en/Reference/SerialEvent
  https://www.arduino.cc/en/Tutorial/SerialEvent
*/
void serialEvent(){
  // Handle incoming serial data from PC, Pioneer LX or Raspberry Pi
  while(Serial.available() && !gotData){                            
    LED_buffer[numBytesRead] = Serial.read();
    numBytesRead++;
    if(numBytesRead == NUM_LEDS*3){  
      gotData = true;               
      numBytesRead = 0;            
    }
/*
 * With a BAUD-RATE of 1Mb/s (or 125 000 bytes/s) the approximated time
 * it enters this clause is NUM_LED*3 / 125 000  which is equal to 14.4 ms with 600 LEDs
 * + 1 ms overhead for additional branch clauses gives 15.4 ms.
 * FAST_LED uses approximatley 30 us to update one pixel, ->so for 600 we have 18 ms delay:
 * giving us a total of 32.4 ms update refresh rate
 */
  }
}
