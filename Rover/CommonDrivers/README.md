# PS2X Lynxmotion Controller Driver

In this repository, we showcase the implementation of a driver for the PS2X Lynxmotion controller. All information has been gathered from what is available on the website [Curious Inventor](https://store.curiousinventor.com/guides/PS2/).

Below, we will provide a brief overview of the communication protocol and associated settings, along with a concise summary of the implemented functionalities.

## Interfacing
The PS2 controller behaves as a slave device, communicating with the master through the SPI protocol. In particular, after various configurations, the following settings are utilized:
- Clock frequency (SCL) 62.5kHz [The operating range should be between 50kHz and 500kHz]
- CPHA = 1: This indicates that data is captured on the rising edge of the clock.
- CPOL = 1: This signifies that the clock is normally high.
The NSS signal (normally high) is essential to initiate communication with the device. Every command sent by the master must be preceded by lowering the NSS signal and followed by a logical high value of NSS.

The adapter used to connect the PS2 controller to the F401RE core board has its nomenclature, which is remapped to the SPI protocol as follows:
- ATT: This pin corresponds to NSS.
- DAT: This pin corresponds to the MISO line.
- COM: This pin corresponds to the MOSI line.

The power supply used in our case was 5V.

## Commands

The controller (slave) interacts with the F401RE board (master) using the SPI protocol in full duplex mode. Each 8-bit sequence sent from the master to the slave is followed by its 8-bit response.

Every command starts with the sequence '0x01,' to which the controller responds with '0xff.' The second byte is crucial and provides numerous pieces of information. It should be interpreted as follows:
- The high nibble can only take two values, '0x7X' or '0x4X.' In the first case, the controller operates in analog mode (the analog LED on the controller is on), and in the second case, the controller is in 'digital' mode (the LED on the controller is off).
- The lower nibble indicates how many groups of 16 bits will follow the header. The header of any message has a size of 3 bytes.

Details on commands and their meanings can be found on the previously linked website.

## Implemented Functionalities
- It is possible to set the controller in analog or digital mode.
- In digital mode, you can read the state of each button (pressed or not pressed) through the function ...
- In analog mode, you can read the position of the joystick levers in a range of values from 0 to 127 along the x and y axes, as well as the state of each button's pressure (still in digital mode).

## Unimplemented Functionalities
For our purposes, it was not necessary to fully implement the communication protocol. We did not consider the option to activate the motors for controller vibrations, nor did we configure the L3 and R3 buttons.

# Generating Documentation
To generate the necessary documentation, you need to install Doxygen. After installation, use the command 'doxygen Doxyfile.'
