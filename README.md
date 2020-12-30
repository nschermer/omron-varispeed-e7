# omron-varispeed-e7

Arduino project to communicate with OMRON Varispeed E7 VFD using memobus for my lathe.

## Reason
I have this VFD hooked up to my lathe, because it makes using the lath much nicer. The lathe also has an electric brake which can be controlled with the VFD (relay). But depending on the job you want to use the mechanical brake (instant stop for thread cutting for example) or decelerate with the VFD (most other work).

To change this, you need to enter the programming menu and change the stopping mode (b1-03), which is doable, but not sexy.

As a small addition I've also added a mode for thread cutting, where you can *boost* the reverse speed, compared to the forward speed. This way you can do a quick return after a long thread cut :sunglasses:.

Furthermore the VFD status is displayed on the LCD and some extra info like the motor power and DC bus voltage.

## MEMOBUS
Memobus is properly described in the manual, it works like RS-485, bit I could not get this working with the existing Arduino libraries, so wrote some code for what was needed to read/write registers in the Varispeed drive.

## Hardware

- OMRON Varispeed E7 (any version will work);
- Arduino pro micro (ATmega32U4 chip, because you can use the serial port for modbus, without interfere the usb-serial connection);
- MAX485 TTL to RS485 module;
- LCD 2004 with I2C interface;
- 20K potentiometer  (10K will work fine as well, but will consume a bit more power) for 10-100 Hz;
- Rotary encoder + button for navigating the menu; 
- Power supply: note that modbus on the MAX485 needs > **5.5V** to avoid communication errors. Therefore the MAX is connected to the RAW pin on the Arduino.

## Layout

![Board layout](https://raw.githubusercontent.com/nschermer/omron-varispeed-e7/main/drawing.png)

## Setup
- Configure the VFD according to your e-motor;
- Set S1 pin 1 to on for RS-485;
- Configure the MEMOBUS settings according to the settings in the memobus.h header (most are the defaults);
- Speed reference from the VFD using memobus;
- I use the 2-wire setup for forward/reverse, but that up to you;
- Make sure the VFD settings in the main ino match you needs (min/max freq, motor poles, units);
- Don't bother me if it doesn't work.

## How it works
- With the potentiometer you can set the motor speed.
- Push the rotary button to cycle the 3 mode: decelerate, brake (coast down mode on vfd), threading (quick reverse).
- Rotate encoder to change threading reverse speed in this mode.
