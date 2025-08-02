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

# Key Functions

## shell.c

```getsUart0```

Gets a character from UART and uses ASCII values to ensure it meets the necessary inputs for the commands defined in main. In the while loop, the function ensures backspaces are ignored, recognizes when a space is entered and ends the string or if a carraige return is sent it ends the string and returns the function.
```
void getsUart0(USER_DATA *data)
{
    int count = 0;
    while(true)
    {
        char letter = getcUart0();

        if(letter >= 32 && letter != 127)
        {
            data->buffer[count] = letter;
            count++;
            if(count == MAX_CHARS)
            {
                data->buffer[MAX_CHARS] = 0;
                break;
            }
        }
        else if(letter == 8 || letter == 127)
        {
            if(count > 0)
            {
                count--;
            }
        }
        else if(letter == 13)
        {
            data->buffer[count] = 0;
            break;
        }
    }
}
```
```parseFields```

Using 3 sets of characters (alpha, numeric, and delimiter) the function assume the last character was a delimiters when searching the buffer and labels the field according to which character set it falls into, this is done until the end of the buffer string is found or until MAX_FIELDS are reached and returns.
```
void parseFields(USER_DATA *data)
{
    int counter = 0;
    char prev_char = 'd';
    int i = 0;
    data->fieldCount = 0;
    for(i = 0; data->buffer[i] != '\0'; i++)
    {
        if(data->fieldCount < MAX_FIELDS)
        {
            if((data->buffer[i] > 47 && data->buffer[i] < 58) || data->buffer[i] == 45 || data->buffer[i] == 46)      //numbers
            {
                if(prev_char == 'd')
                {
                    data->fieldPosition[counter] = i;
                    data->fieldCount++;
                    data->fieldType[counter] = 'n';
                    counter++;
                }

                prev_char = 'n';

            }
            else if((data->buffer[i] > 64 && data->buffer[i] < 91) || (data->buffer[i] < 123 && data->buffer[i] > 96))  // alpha
            {
                if(prev_char == 'd')
                {
                    data->fieldPosition[counter] = i;
                    data->fieldCount++;
                    data->fieldType[counter] = 'a';
                    counter++;
                }
                prev_char = 'a';
            }
            else
            {
                data->buffer[i] = '\0';
                prev_char = 'd';
            }

        }
        else
        {
            return;
        }

    }

}
```
```getFieldString```

Returns the value of the field requested if the field number is in range otherwise returns NULL.
```
char* getFieldString(USER_DATA *data, uint8_t fieldNumber)              //returns pointer to where string exist
{
    char *ret;

    if((fieldNumber <= data->fieldCount || fieldNumber == 0) && data->fieldType[fieldNumber] == 'a')
    {
        ret = &data->buffer[data->fieldPosition[fieldNumber]];
    }
    else
    {
        return NULL;
    }

    return ret;

}
```
```getFieldInteger```

Returns the integer value of the field if the field number is in range and the field type is numeric otherwise returns NULL
```
int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{

    int32_t ret;

    if((fieldNumber <= data->fieldCount || fieldNumber == 0) && data->fieldType[fieldNumber] == 'n')
    {
        ret = atoi(&data->buffer[data->fieldPosition[fieldNumber]]);
    }
    else
    {
        return NULL;
    }

    return ret;

}
```
```isCommand```

This function returns true if the command matches the first field and the number of arguments is greater than or equal to the requested number of minimum arguments.
```
bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments)
{
    bool ret = 0;
    int counter = 0;
    int pos = data->fieldPosition[0];
    if( (data->fieldType[data->fieldPosition[0]] == 'a') && ( (data->fieldCount - 1) >= minArguments) )
    {
        while(data->buffer[counter] != '\0')
        {
            if(data->buffer[pos] == strCommand[counter])
            {
                counter++;
                pos++;
                ret = 1;
            }
            else
            {
                ret = 0;
                break;
            }
        }
    }
    else
    {
        ret = 0;
    }
    return ret;
}
```
## main.c

```initPWM```

This function initilizes the PWM signals for motor control and configures GPIO port C for PWM output and GPIO B and E for motor direction
```
void initPWM()
{
    GPIO_PORTC_DEN_R |= ENA_MASK | ENB_MASK;
    GPIO_PORTC_AFSEL_R |= ENA_MASK | ENB_MASK;                     //General I/O Pin
    GPIO_PORTC_PCTL_R &= ~(GPIO_PCTL_PC4_M | GPIO_PCTL_PC5_M);              //Clear PCTL
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC4_M0PWM6 | GPIO_PCTL_PC5_M0PWM7;

    GPIO_PORTB_DIR_R |= IN1_MASK | IN2_MASK;
    GPIO_PORTE_DIR_R |= IN3_MASK | IN4_MASK;
    GPIO_PORTB_DEN_R |= IN1_MASK | IN2_MASK;
    GPIO_PORTE_DEN_R |= IN3_MASK | IN4_MASK;

    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;                   // reset PWM0 module
    SYSCTL_SRPWM_R = 0;                                 // leave reset state
    PWM0_3_CTL_R = 0;                                   //M0 Gen 3
    // output 6 on PWM0, gen 3A, cmpa
    PWM0_3_GENA_R = PWM_0_GENA_ACTCMPAD_ONE | PWM_0_GENA_ACTLOAD_ZERO;
    // output 7 on PWM0, gen 3A, cmpb
    PWM0_3_GENB_R = PWM_0_GENB_ACTCMPBD_ONE | PWM_0_GENB_ACTLOAD_ZERO;      // turn-off PWM0 generator 3
    PWM0_3_LOAD_R = 19999;                                       // set frequency to 40 MHz sys clock / 2 / 20000 = 1 kHz
    PWM0_3_CMPA_R = 0;
    PWM0_3_CMPB_R = 0;
    PWM0_3_CTL_R = PWM_0_CTL_ENABLE;                            // turn-on PWM0 generator 3
    PWM0_ENABLE_R = PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
}
```
```Timer1A_ISR```

This is the interrupt service routine for Timer 1A where the IMU data is read to get the latest accelerometer and gyroscope data. The raw accelerometer data is used to calculate the current pitch angle of the robot. The calculated pitch is then fused with the gyroscope data using a complimentary filter which results is a more stable and accurate pitch reading. A PD controller is then implemented to calculate the necessary motor response using the current pitch angle and the rate of change of the pitch angle.
```
void Timer1A_ISR()                  //Left Wheel
{
    TIMER1_ICR_R |= TIMER_ICR_TATOCINT;
    if(balance)
    {
        GPIO_PORTF_DATA_R ^= GREEN_LED_MASK;
        int16_t last_pitch = 0;
        int16_t filt_der = 0;
        readIMU((int16_t*)&imu_data.ax, (int16_t*)&imu_data.ay, (int16_t*)&imu_data.az,
                (int16_t*)&imu_data.gx, (int16_t*)&imu_data.gy, (int16_t*)&imu_data.gz);
        int16_t accelpitch = computePitch(imu_data.ax, imu_data.az);
        updatePitch(accelpitch, imu_data.gy);
        if(abs(pitch) < 100 )
        {
            PWM0_3_CMPA_R = 0;
            PWM0_3_CMPB_R = 0;
            return;
        }
        int16_t derivative = pitch - last_pitch;
        last_pitch = pitch;
        filt_der = (filt_der * 7 + derivative) / 7;             //87.5% filter
        int32_t motor = (pitch * Kp - filt_der * Kd) / 100;


        int16_t direction = (motor > 0) ? 1 : -1;
        int16_t mag = abs(motor);
        if(mag > 4000)
            mag = 4000;


        int16_t duty = 14000 + mag;

        if(direction > 0)
        {
            GPIO_PORTB_DATA_R &= ~IN1_MASK;     //Backward
            GPIO_PORTB_DATA_R |= IN2_MASK;
            GPIO_PORTE_DATA_R &= ~IN3_MASK;
            GPIO_PORTE_DATA_R |= IN4_MASK;
        }
        else
        {
            GPIO_PORTB_DATA_R |= IN1_MASK;      //Forward
            GPIO_PORTB_DATA_R &= ~IN2_MASK;
            GPIO_PORTE_DATA_R |= IN3_MASK;
            GPIO_PORTE_DATA_R &= ~IN4_MASK;
        }
        PWM0_3_CMPA_R = duty;
        PWM0_3_CMPB_R = duty;
    }

}
```
```computePitch```

This function calculates the pitch angle in degrees based on the raw accelerometer data ```ax``` and ```az```). It approximates the tangent of the pitch angle and then converts it to degrees.
```
int16_t computePitch(int16_t ax, int16_t az)
{
    if(az == 0)
    {
        az = 1;
    }
    int32_t ratio = ((int32_t)ax * 1000)/az;    //Approximates tangent of pitch angle
    if(ratio > 3000)
    {
        ratio = 3000;
    }
    else if(ratio < -3000)
    {
        ratio = -3000;
    }
    return (int16_t)((ratio * 57)/1000);        //converts radian to degrees
}
```
```updatePitch```

This function implements a complementary filter to combine the accelerometer-derived pitch with the gyroscope's angular velocity data. This provides a more stable and accurate estimate of the pitch angle. The filter gives more weight to the gyroscope data for short-term changes and uses the accelerometer data to correct for drift over the long term.
```
void updatePitch(int16_t accel_deg, int16_t gyro_raw)
{
    gyro_raw -= gyro_offset;
    uint8_t alpha = 92;         //trust gyro 92%
    uint16_t dt = 10;        //loop period 10ms
    int16_t gyro_s = 65;   //ICM gyro LSB/deg/s    Page 11

    int32_t gyro_scaled = ((int32_t)gyro_raw * 100) / gyro_s;   //converts to deg/s
    int16_t delta = (gyro_scaled * dt) / 10;
    int16_t gyro_pitch = pitch + delta;
    pitch = ((gyro_pitch * alpha) + (accel_deg * (100 - alpha))) / 100;
}
```
## How to Use
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
