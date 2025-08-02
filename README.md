# Self-Balancing Robot

## Overview 
This repository contains the code for a two-wheeled self-balancing robot based on the TM4C123GH6PM Tiva C Series microcontroller. The robot utilizes an ICM-20948 9-axis IMU for motion sensing, an L298N Motor Driver for controlling two DC motors, and is powered by a 4 AA battery pack. The project demonstrates real-time embedded control, sensor fusion, and precise PWM motor actuation to maintain balance.

## Features
- Self-Balancing: The robot uses a PD(Proportional-Derivative) controller to maintain its upright position.
- Real-time Sensor Fusion: Data from the accelerometer and gyroscope are combined to accurately determine the robot's tilt angle.
- Motor Control: PWM signals are used to drive the DC motors and keep the robot balanced.
- Serial Communication: The robot can be controlled and monitored via a UART interface using a terminal emulator in this project PuTTY is used.

## Hardware
- Microcontroller: TM4C123GH6PM Tiva C Series
- IMU: ICM-20948 9-axis (accelerometer and gyroscope)
- Motor Driver: L298N
- Motors: Two DC motors
- Power Source: 4 AA Battery Pack

## Software
This project is written in C and contains the following files:
- ```main.c ```: The main application file that initializes the hardware, implements the balancing algorithm, and handles user commands.
- ```shell.c```: Contains functions for parsing and interpreting commands received over UART.
- ```uart0.c / uart0.h```: Drivers for UART0 communication.
- ```clock.c / clock.h```: Clock configuration for the microcontroller.
- ```wait.c / wait.h```: Functions for creating delays.
- ```tm4c123gh6pm.h```: Header file with register definitions for the TM4C123GH6PM.
- ```tm4c123gh6pm_startup_ccs.c```: Startup file for Code Composer Studio.

## Key Functions
- ```initUart0()```: Initializes the UART0 module for serial communication.
- ```initPWM()```: Configures the PWM module for motor control.
- ```initI2C0()```: Initializes the I2C0 module for communication with the IMU.
- ```readIMU()```: Reads accelerometer and gyroscope data from the ICM-20948.
- ```computePitch()```: Calculates the robot's pitch angle based on accelerometer data.
- ```updatePitch()```: Fuses accelerometer and gyroscope data to get a more accurate pitch angle.
- ```Timer1A_ISR()```: The interrupt service routine for Timer 1A, which runs the balancing algorithm at regular intervals.

##How to Use
1. Hardware Setup: Connect the ICM-20948 IMU to the I2C0 pins (PB2 for SCL and PB3 for SDA) of the TM4C123GH6PM. Connect the L298N motor driver to the PWM and GPIO pins as defined in main.c. Power the microcontroller and motor driver with the 4 AA battery pack.
2. Software Setup:
- Open the project in Code Composer Studio.
- Build the project to generate the executable file (.out).
- Flash the .out file to the TM4C123GH6PM.
3.Running the Robot:
Connect to the robot's virtual COM port using a terminal emulator (e.g., PuTTY) with a baud rate of 115200.
Use the following commands to control the robot:
-startup: The robot will move forward and backward to demonstrate motor functionality.
-balance: The robot will start the self-balancing algorithm.
-stop: The robot will stop all movement.
-angle: Prints the raw and filtered pitch angle to the terminal.
