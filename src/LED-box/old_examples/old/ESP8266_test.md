# ESP8266

**The ESP8266 has been replaced by the Arduino Mega. Disregard this file.**

For running the LEDs without signals from the neurons, a test show simulating random inputs from the neurons is written.

ROS interface (LED_driver)

A node running on the Raspberry Pi serves as the ROS interface: It parses the ROS messages and transmits them to the ESP8266 inside the head of the robot. The ESP8266 then translates this message to LED states, and sets the lights. As of today, the ROS node accepts message of type "LED_driver::LedCommandArray". This is a list of 240 "LED_driver::LedCommand" messages. The LedCommand-message simply contains an uint8. For details on why these as set up like this, please consult the project report (EiT, 2017, group 2). Each number must be 0-9, as described in the following table.
- 0	Off
- 1	White
- 2	Red
- 3	Light red
- 4	Yellow
- 5	Light yellow
- 6	Green
- 7	Light green
- 8	Blue
- 9	Light blue
