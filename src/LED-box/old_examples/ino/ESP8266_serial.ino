#include <FastLED.h>
#define NUM_LEDS 240
#define DATA_PIN 6
#define NUM_PARAMS 3
#define NUM_COLORS 10
#define ASCII_OFFSET 48

/*
Arduino code for the nodeMCU ESP8266 developed by EiT. The argument for the ESP8266 is that it has wifi. 
The problem is the wifi can not connect to Eduroam. 
The ESP8266 also had some problem with the maximum voltage out of its I/O ports (see EiT report).
*/

CRGB leds[NUM_LEDS];
enum light_parameter{HUE=0, SATURATION, VALUE};
uint8_t color_map[NUM_COLORS][NUM_PARAMS] = {{0,0,0},
                                          {0,0,255},
                                          {HUE_RED
                                          ,255,255},
                                          {HUE_RED
                                          ,128,255},
                                          {HUE_YELLOW
                                          ,255,255},
                                          {HUE_YELLOW
                                          ,128,255},
                                          {HUE_GREEN
                                          ,255,255},
                                          {HUE_GREEN
                                          ,128,255},
                                          {HUE_BLUE
                                          ,255,255},
                                          {HUE_BLUE
                                          ,128,255}};
                                          
void readFromSerial(){
  uint8_t index=0;
  char current_char=0;
  while(current_char != '\n' && index < NUM_LEDS){
    if(Serial.available()){
      current_char = Serial.read();
      if(current_char != '\n'){
        uint8_t parameter_value = current_char - ASCII_OFFSET;
        leds[index++] = CHSV( color_map[parameter_value][HUE], 
                              color_map[parameter_value][SATURATION], 
                              color_map[parameter_value][VALUE] );
      }
    }
  }
}

void startupFade(uint8_t hue, uint8_t saturation, uint8_t fade_time){
  for(int value = 0; value < 156; value++){
    FastLED.showColor(CHSV(hue, saturation, value));
    delay(fade_time);
  }
  for(int value = 156; value >= 0; value--){
    FastLED.showColor(CHSV(hue, saturation, value));
    delay(fade_time);
  }
}

void setup(){
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  startupFade(HUE_BLUE, 255, 3);
  Serial.begin(115200);
  while(Serial.available()){
    Serial.read();//Flushserial
  }
}

void loop(){
  if(Serial.available()){
    readFromSerial();
    FastLED.show();
  }
  delay(1);
}
