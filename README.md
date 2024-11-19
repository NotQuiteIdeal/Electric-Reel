# Electric-Reel
This project aims to variety of advanced features into a repurposed reel to provide users with more accurate data, intuitive controls, and enhanced performance for deep-sea fishing.
### Main Reel
The main reel will be central to the whole project. All data processing will happen on the microcontroller in the reel. The main reel will incorporate an LCD that will display reel information, such as drag and line lentgh, and allow the user to control the motor. The main reel will also communicate with the mobile device to allow wireless control from said device.
### User Interface
This module focuses on the layout and design of the user inputs and outputs on the main reel system. The goal is to create an intuitive and functional interface that allows users to easily adjust settings, view important data, and receive alerts while out on the water. 

Components Included:
-LCD Screen: Displays important information such as the current drag setting and line length.
-Rotary Encoder: Used for navigating menus, selecting options, and adjusting settings, particularly useful for entering values.
-Buttons: Two buttons allow users to access a settings menu and navigate through various options.
-Buzzer: Provides an audible fish alert when a fish is detected on the line.
### Line Length and Drag
Encoders will measure both the line length and drag set. An encoder placed on the axle that rotates as the line guide moves will send pulses to the microcontroller, which will then use code to convert the values to a readable measurement on the LCD display. Line Length accuraccy must fall within +/- 15 ft at 200 ft. 
### Pulse Width Modulation (PWM)
To provide variable speed control for the reel motor, the D.D. Reels project will utilize an H-Bridge circuit. This H-Bridge will be driven by a 3.3V Pulse Width Modulated (PWM) signal from the microcontroller. The PWM signal allows for precise speed control by adjusting the duty cycle, which in turn controls the power sent to the motor.
### Mobile Device
To meet the functional and communication needs of both the main reel and the mobile device, the system requires two microcontrollers. The mobile device will fetch information from the main reel, line length and drag, to desplay it on its own LCD display. It will also be able to activate the reel in on the main device and include a fish alert buzzer. 

Main Reel Microcontroller:
-Acts as the "server" in the Bluetooth LE communication setup.
-Handles the calculations for critical data, including line length, drag settings, and motor status.
-Stores additional real-time data such as the fish alarm status and motor control information.
-Communicates data updates to the mobile device every 200ms via notifications.

Mobile Device Microcontroller:
-Acts as the "client" in the Bluetooth LE setup.
-Requests and receives data updates from the main reel, including line length, drag settings, and other critical information.
-Sends write requests to the main reel when the user adjusts settings or controls, such as changing drag values or starting/stopping the motor.
