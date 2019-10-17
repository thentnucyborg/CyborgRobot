/*
  https://playground.arduino.cc/Interfacing/Python

  Using FastLED library:
  - Download the FastLED library: http://fastled.io/ as .zip.
  - In Arduino IDE: Sketch -> Include library -> Add .ZIP library
*/
#include <FastLED.h>
#define NUM_LEDS 240
#define DATA_PIN 6

// INITIATORS FOR SERIAL COMMUNICATION
char dataString[50] = {0};
int a =0; 
int msg;
String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete

// INITIATORS FOR LEDS AND LED UPDATE MODE
CRGB leds[NUM_LEDS];
enum colorcode{RED=0, GREEN, BLUE};
enum mode{RANDOM=0, FILL_ALL_LOOP, FILL_ONE_LOOP, HORISONTAL_LOOP, SERIAL_INPUT};
int set_mode;
String colorStr = ""; 

// COLOR FILLER FUNCTIONS
void fillAll(CRGB color){
  uint8_t i = 0;
  while(i < NUM_LEDS){
    leds[i] = color; //CHSV(r, g, b);
    i++;
  }
  FastLED.show();
}

void fillHorisontal(CRGB color){
  uint8_t i = 0;
  for(uint8_t i = 0; i<NUM_LEDS; i=i+40){ 
    for(uint8_t j = 0; j<40; j++){ 
      leds[i+j] = color; //CHSV(r, g, b);
    }
    FastLED.show();
    delay(100);
  }
  FastLED.show();
}

void fillOne(CRGB color){
  uint8_t i = 0;
  while(i < NUM_LEDS){
    leds[i] = color; //CHSV(r, g, b);
    FastLED.show();
    delay(10);
    i++;
  }
  
}

void fillRandom(){
  uint8_t i = 0;
  while(i < NUM_LEDS){
    int r = random(3);
    if(r == RED){ 
      leds[i] = CRGB::Red;
    }else if(r == GREEN){
      leds[i] = CRGB::Green;
    }else{
      leds[i] = CRGB::Blue;      
    }
    i++;
  }
  FastLED.show();
}

// COLOR LOOP FUNCTIONS
void loopAll(int time_delay){
  fillAll(CRGB::Red);
  delay(time_delay);
  fillAll(CRGB::Blue);
  delay(time_delay); 
  fillAll(CRGB::Green);
  delay(time_delay); 
}

void loopOne(){
  fillOne(CRGB::Red);
  fillOne(CRGB::Blue);
  fillOne(CRGB::Green);
}

void loopHorisontal(){
  fillHorisontal(CRGB::Red);
  fillHorisontal(CRGB::Blue);
  fillHorisontal(CRGB::Green);
  fillHorisontal(CRGB::Yellow);
}

// OTHER COLOR FUNCTIONS
void pulse(uint8_t hue, uint8_t saturation, uint8_t fade_time){
  for(int value = 0; value < 156; value++){
    FastLED.showColor(CHSV(hue, saturation, value));
    delay(fade_time);
  }
  for(int value = 156; value >= 0; value--){
    FastLED.showColor(CHSV(hue, saturation, value));
    delay(fade_time);
  }
}

void startupFade(uint8_t fade_time){
  pulse(HUE_BLUE, 255, fade_time);
  pulse(HUE_RED, 255, fade_time);
  pulse(HUE_GREEN, 255, fade_time);
  pulse(HUE_YELLOW, 255, fade_time);
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
  while(Serial.available()){
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

// SETUP AND MAIN LOOP
void setup(){
  Serial.begin(9600);              //Starting serial communication
  inputString.reserve(NUM_LEDS);
  randomSeed(analogRead(0));
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  startupFade(1);
  set_mode = RANDOM;
}

void loop(){
  switch(set_mode){
    case RANDOM:
      fillRandom();
      break;
    case FILL_ALL_LOOP:
      loopAll(200);
      break;
    case FILL_ONE_LOOP:
      loopOne();
      break;
    case HORISONTAL_LOOP:
      loopHorisontal();
      break;
    default:
      fillRandom();
    break;
  }

  // Handle incoming serial data from PC, Pioneer LX or Raspberry Pi
  if(stringComplete){
    Serial.println(inputString);
    if(inputString == "start_serial_input"){
      set_mode = SERIAL_INPUT;
    }
    if(set_mode == SERIAL_INPUT){
      colorStr = inputString; // convert a 240 char string into a 240 LED color update. 
    }
    inputString = "";
    stringComplete = false;
  }
  
  delay(100);
}


