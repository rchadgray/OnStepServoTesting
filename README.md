# OnStepServoTesting

THis code has my wifi password in it.... dont make it public

############## START notes for DC Servo #############################

https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
Use ESP32 board manager 2.0

http://arduino.esp8266.com/stable/package_esp8266com_index.json
Use 8266 Board manager 2.4.2

##############  END notes for DC Servo #############################


############# START notes for Stepper Encoder ########################
There were a few typos that Howard fixed to get code to compile with the SERVO_TMX2209 driver

Current bug is the Steppers never enable.

Encoder data is being read by the encoder bridge.

Stepper motor came with EA+ EA- and EB+ EB-   I only used EA+ and EB+ connected to the Encoder Bridge AB


############# END notes for Stepper Encoder ########################