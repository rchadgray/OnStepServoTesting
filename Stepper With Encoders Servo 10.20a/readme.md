############# START notes for Stepper Encoder As Servo ########################

This physical setup is NEMA17 200 Stepper motor that has an incremental A/B encoder built in.
https://www.omc-stepperonline.com/nema-17-closed-loop-stepper-motor-52ncm-73-64oz-in-with-encoder-1000ppr-4000cpr-17hs19-2004d-e1k

1275:1 gear train after stepper motor

1000 p/r or 4000 c/r A/B Quad incremental encoder attached to motor.  So if motor turns 1 revolution then ecoder turns 1 revolution.

Counts per degree math:  1275 / 360 = 3.5417 Motor Turns Per degree of telescope movement
3.5417 * 4000 c/r = 14166.67 counts per degree of telescope movement

MaxESP4 PCB with TMC2209 UART drivers

https://oshwlab.com/hdutton/db15exp-abd




