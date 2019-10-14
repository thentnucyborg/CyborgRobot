#include <FastLED.h>
#define NUM_LEDS 240
#define DATA_PIN 6

CRGB leds[NUM_LEDS];

enum colorcode{RED=0, GREEN, BLUE};

void fillAll(CRGB color){
  uint8_t i = 0;
  while(i < NUM_LEDS){
    leds[i] = color; //CHSV(r, g, b);
    i++;
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

void randomFill(){
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

void rgbLoopAll(int time_delay){
  fillAll(CRGB::Red);
  delay(time_delay);
  fillAll(CRGB::Blue);
  delay(time_delay); 
  fillAll(CRGB::Green);
  delay(time_delay); 
}

void rgbLoopOne(){
  fillOne(CRGB::Red);
  fillOne(CRGB::Blue);
  fillOne(CRGB::Green);
}

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

void setup(){
  randomSeed(analogRead(0));
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  //startupFade(1);
}

void loop(){
  rgbLoopOne();
  //randomFill();
  //delay(1000);
}
