# Home-Brewing Automation with an Arduino Nano
Arduino Nano firmware for my home-brewery

![frontpanel](img/main_pcb_picture.jpg)<br>
*Top view of v3.30 of the main-PCB*

# Features
The current PCB and firmware have the following features:
- Reading a maximum of 4 temperature sensors: one each for the three brewing-kettles and one at the output of the counterflow-chiller
- Reading of the hardware temperature: this is to protect the power-electronics
- Reading a maximum of 4 flowsensors: between HLT and MLT, between MLT and boil-kettle and one at the output of the counterflow-chiller
- Control of 8 solenoid ball-valves at 24 V DC
- Control of 4 actuators at 230 V AC: the pump, a second pump (for the HLT counterflow-chiller), the HLT gasburner and the boil-kettle gasburner

More information about hardware-architecture and actual PCB-design: see my website: http://www.vandelogt.nl/uk_hardware.php

# Software Development Environment
Use with Atmel Studio v6 or higher.

# Interface with PC
The Arduino-Nano uses a virtual COM port as its main-connection to the PC. This virtual COM port used the USB port of the Arduino-Nano. At the PC side, the Arduino Nano is recognised by Windows
(if the standard drivers for the Arduino-Nano have been installed). Virtual COM port settings are ()38400,N,8,1).

Typically the PC-program sends commands to the Arduino-Nano, like **P0** (Pump Off) or **P1** (Pump On). These commands are then executed by the Arduino-Nano.
Although you can type in the commands manually, it is more efficient to use a dedicated PC-program for it, with a nice Graphical User Interface.

More information about the PC-Interface can be found at my website: http://www.vandelogt.nl/uk_hardware.php#PC_INTF



