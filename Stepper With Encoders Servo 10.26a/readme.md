############# START notes for Stepper Encoder As Servo ########################

This physical setup was NEMA17 200 Step motor
1275:1 gear train after stepper motor
600 p/r or 2400 c/r A/B Quad incremental encoder attached to motor output.  So if motor turns 1 revolution then ecoders turns 1 revolution.
https://www.amazon.com/Taiss-Incremental-Encoder-Voltage-Warranty%EF%BC%89600P/dp/B07MX1SYXB/?th=1

Counts per degree math:  1275 / 360 = 3.5417 Motor Turns Per degree of telescope movement
3.5417 * 2400 c/r = 8500.08 counts per degree of telescope movement

MaxESP4 PCB with TMC2209 UART drivers

9-3-25 ERROR!  No tracking or slews possible.
Running into a Driver/Motor fault



