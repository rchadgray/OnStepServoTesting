# OnStepServoTesting
My running formulas to generate the Counts Per Degree and Steps Per Degree
https://docs.google.com/spreadsheets/d/1Tq_renhxYmt1QZXeV3cT8AOJsoXvolBXulJBMp35MlY/edit?usp=sharing

This collection of code is my testing of the MaxESP4 with Encoders. Below are my use cases.

-----------------
DC Servo Morots
Hardware Used:
https://onstep.groups.io/g/main/wiki/33520

https://www.amazon.com/dp/B07GNGGCVP?th=1

https://oshwlab.com/hdutton/dcstick 

This code is my first attempt at using the MaxESP4 with DC Stick modules.

-------------------
Steppers with Encoders for Pointing
Hardware used:
https://onstep.groups.io/g/main/wiki/33520

https://oshwlab.com/hdutton/db15exp-ab

https://www.amazon.com/Taiss-Incremental-Encoder-Voltage-Warranty%EF%BC%89600P/dp/B07MX1SYXB/?th=1

This code uses encoders not in "servo mode" but pointing.  The encoders read the pointing of the mount/telescope.  If the motor position and encoder positions deviate you can see this in the Smart Web Server.  There is an option to sync the encoders to OnStep or Sync OnStep to the encoders.  This is a great place to start with encoders as you get to learn the wiring of the encoder to make them successful and see a visual display of encoder positioning vs. stepper.

---------------------
Steppers with Encoders Servo (10.20a is successful others are experimental)
Hardware Used:
https://onstep.groups.io/g/main/wiki/33520

https://oshwlab.com/hdutton/db15exp-abd 

https://www.omc-stepperonline.com/nema-17-closed-loop-stepper-motor-52ncm-73-64oz-in-with-encoder-1000ppr-4000cpr-17hs19-2004d-e1k

This code is using a stepper motor with build in encoder.  OnStep uses the counts of the encoder to know it's position.  This is servo mode in OnStep.






