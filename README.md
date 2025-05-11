This repository contains the STM32CubeIDE project files for the author's bachelor's thesis on the topic "Development of an Arc Extinguisher Mechanism and Soft-Start Methodology for a DC electric kettle"

All the code written directly by the author can be found in Core/Src/main.c

The ECUAL folder contains the libraries for I2C used to control an LCD screen.

The main program itself is separated into four enum states - KETTLE_IDLE, KETTLE_TURN_ON, KETTLE_TEST and KETTLE_TURN_OFF.

In KETTLE_IDLE, the program waits for input from the user (using a physical interface on the kettle) to turn the kettle on.

In KETTLE_TURN_ON, the program switches on a relay to precharge the input capacitor. This part of the code also turns on the SSCB.

In KETTLE_TEST, the program measures the input and output current and voltage as well as the water temperature (using a temp sensor).
The error states for overcurrent are also within this block of code.
KETTLE_TEST also contains calculations for the duty cycle value.

In KETTLE_TURN_OFF, the program gradually turns off the DC kettle's buck converter by reducing the transistors' duty cycle value to 0.

The duty cycles are assigned using a timer interrupt.

The user interface and arc extinguisher buttons are also managed using EXTI channels.






