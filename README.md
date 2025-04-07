# Sound Tracker

<img src="pictures/soundTracker.gif" width=500>

# Overview

This project incorporates embedded programming and digital signal processing to determine the direction a sound is coming from and then point in that direction.

three microphones are spaced a foot apart and used to sample sounds moving across their positions. A microcontroller is programmed to use ADC to record the moment each mic hears the same sound. The microcontroller uses the time delay to calculate which direction the sound is coming from. The microcontroller then sends PWM signals to two servo motors to point toward the direction of the sound. Trigonometry and some vector math is used for the calculations.

Please take a look at the following video to see the sound tracker in action.

[Project demo video](https://us06web.zoom.us/rec/share/YqVxKYFJ1pkpHedOYRGfWK72mYiph7xyRjjW_O6RvDqyYby64nobYXEJm1mKRITD.CpcUHFmE6bIQpR7C?startTime=1698197367000)

# Development Environment

The microprocessor is an stm32 Nucleo board. The stm32CubeIDE was used to configure the board.

This project was programmed in C and compiled into ARM32 Assembly.

The microphones are MAX9814 from Electret

the motors were SG90 9g Micro Servos from Beffkkip

# Configuration
Figure 1 shows the schamatic for the project: 

<img src="pictures/soundTrackerDiagram.png" width=500>

Figure 2 shows a screenshot of the stm32CubeIDE GUI used to configure the board for analog and PWM input and output: 

<img src="pictures/iocGUI.png" width=500>

Figure 3 shows the microcontroller and wiring

<img src="pictures/nucleo.jpg" width=500>

Figure 4 shows the complete device

<img src="pictures/device.jpg" width=500>

# Useful Links

- [Microphones on Amazon](https://www.amazon.com/dp/B0B7SP6GYX?ref=ppx_yo2ov_dt_b_fed_asin_title)

- [Servo motors on Amazon](https://www.amazon.com/dp/B07MLR1498?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1)

# Future Work
Get bigger motors, attach a toy nerf gun, and have it shoot my roommates when they get too loud.

