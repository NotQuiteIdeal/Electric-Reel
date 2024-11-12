# Electric-Reel
This project aims to create an electric reel powered by a Raspberry Pi Pico W that can be controlled remotely with a mobile device also powered by a Raspberry Pi Pico W.

## Main Reel
The main reel will be central to the whole project. All data processing will happen on the microcontroller in the reel.
### User Interface
A single display will be installed on the reel which will display the line length, drag set, and settings. The line length and drag set will appear on the main (default) screen and update at least 5 times a second. The settings can be accessed by pressing the encoder, and each setting can be navigated with the dial on the encoder and the buttons on each side of the encoder.
### Line Length and Drag
Encoders will measure both the line length and drag set. An encoder placed on the axle that rotates as the line guide moves will send pulses to the microcontroller
### Pulse Width Modulation (PWM)


## Mobile Device

