# Arduino code
The Arduino recieves data on serial and sets the LEDs on the LED box.

## Files overview
There are currently 3 implementations. ArduinoMega OR ArduinoMega_serial should be used.

### ArduinoMega
Arduino code for uploading simple patterns without the need for any other devices other than the Arduino. 

### ArduinoMega_serial
An extension of the above code, including code for listning to serial. This enables a PC, the Pioneer LX or Raspberry Pi to send commands to the Arduino. These devices can then host an MEA client which can recieve neural data and transform into LED instructions to the Arduino by serial.

### Kybrog_FastLED
Created by EiT group V2018.

### ESP8266_serial
Old code for the ESP8266 board. Replaced by the Arduino Mega.