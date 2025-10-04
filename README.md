# DRV8825Motor

NEMA17 with DRV8825 and 28BYJ-48 Stepper Motor with ULN2003

There is one sketch and with a #define the motor type / driver is selected.
A rotary encoder controls the microstepper mode and rotation speed. 

```
ULN2003 
Driver Board	Connection
IN1	            Pin 8 Arduino
IN2	            Pin 9 Arduino
IN3	            Pin 10 Arduino
IN4	            Pin 11 Arduino
–	            GND power supply
+	            5 V power supply
```
The motor power supply is 5V and be connected to + and GND to -. You can use a USB 5V power supply of 1A
Stepper motor coils are connected with a connector.

The Arduino is powered via the USB-cable
IN1-IN4 are connected to pin 8, 9 ,10 and 11



The rotary encoder is connected to Arduino pin 2, CLS, 3 DT, 4 SW, + to 5v and - to GND
```
CLK	            Pin 2 Arduino
DT 	            Pin 3 Arduino
SW 	            Pin 4 Arduino
–	            GND pin Arduino
+	            5V  pin Arduino
```
<img alt="NEMA DRV8825"  src="DRV8825_files/image009.png" width="450" /><br />

height="436"
```
DRV8825 Connections
DRV8825	Connection
VMOT	8 - 45 V
GND	    Motor ground
SLP	    5 V
RST	    5 V
GND	    Logic ground
STP	    Pin 3
DIR	    Pin 2
A1, A2, B1, B2	Stepper motor
```
The motor power supply shall be connected to VMOT and GND (top right).
Stepper motor coils shall be connected to A1, A2, B1, and B2 as indicated in the pinout diagram.

The GND pin (bottom right) must be connected to the microcontroller ground reference. VDD shall be connected to the 5 V logic supply.
The STP (step) and DIR (direction) pins are connected to digital pins 3 and 2, respectively. Alternate digital pins may be used if defined accordingly in software.

The RST (reset) and SLP (sleep) pins must be tied to 5 V to enable the driver.
The EN (enable) pin is internally pulled low and may remain unconnected. When driven high, the driver is disabled.

The FAULT output is an open-drain signal that asserts low when the H-bridge FETs are disabled due to overcurrent protection or thermal shutdown. This pin is not utilized in this configuration.
