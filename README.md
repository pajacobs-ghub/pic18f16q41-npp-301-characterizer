Introduction
------------

The C-language firmware runs on the PIC18F16Q41 microcontroller while the
Python monitoring program runs on a PC.
RS485 messages are sent by the Python program and the microcontroller responds
to messages sent to node `N`.

When asked, the microcontroller applies a reference voltage and 
samples the voltages at the 5 connected pins of the NPP-301 sensor.
These voltages are reported directly as 12-bit integers.

The Python program converts these measurements to resistance estimates,
assuming that the bottom two pins of the NPP-301 bridge are connected to ground
through 1k reference resistors.  Resistances are reported in ohms.    
