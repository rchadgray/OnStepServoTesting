# OnStepServoTesting
My running formulas to generate the Counts Per Degree and Steps Per Degree
https://docs.google.com/spreadsheets/d/1Tq_renhxYmt1QZXeV3cT8AOJsoXvolBXulJBMp35MlY/edit?usp=sharing



############## START notes for DC Servo #############################

https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
Use ESP32 board manager 2.0

http://arduino.esp8266.com/stable/package_esp8266com_index.json
Use 8266 Board manager 2.4.2

I found these cheap motors to test with.  I will format all of my code to reflect using these motors on a CG5 mount for consistency.
https://www.amazon.com/dp/B07GNGGCVP


##############  END notes for DC Servo #############################




############# START notes for Stepper Encoder ########################

Encoder data is being read by the encoder bridge.

Stepper motor came with EA+ EA- and EB+ EB-   I only used EA+ and EB+ connected to the Encoder Bridge AB
https://www.amazon.com/gp/product/B08Q7H4MBS

Not getting good stepper motor movements.  Lots of stalling.

############# END notes for Stepper Encoder ########################