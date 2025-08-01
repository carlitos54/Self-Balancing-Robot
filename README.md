# Self-Balancing Robot

## Overview 
A two-wheeled robot capable of continuously self-balancing using real-time sensor feedback and control systems. It is powered by a 4 AA battery pack connected through the device pin of the TM4C123GH6PM Tiva C Series where the robot leverages onboard peripherals including PWM, I2C, and hardware timers.
The design incorporates an ICM-20948 9-axis IMU for motion sensing (accelerometer and gyroscope), a L298N Motor Driver for dual DC motor control via PWM, a 4 AA Battery Pack as the main power source.
Using I2C communication, the TM4C reads sensor data from the ICM-20948. A fixed-point PD (Proportional-Derivative) controller runs at regular intervals using a timer interrupt to process tilt angle and angular velocity, then generates motor commands to keep the robot upright. This project showcases real-time embedded control, sensor fusion, and precise PWM motor actuation.

# File Content
## shell.c
Contains functions necessary for robot control and commands by parsing and interpretting data being sent over UART through the puTTY terminal
### getsUart0
Gets a character from UART and uses ASCII values to ensure it meets the necessary inputs for the commands defined in main. In the while loop, the function ensures backspaces are ignored, recognizes when a space is entered and ends the string or if a carraige return is sent it ends the string and returns the function.
```c
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
### parseFields
Using 3 sets of characters (alpha, numeric, and delimiter) the function assume the last character was a delimiters when searching the buffer and labels the field according to which character set it falls into, this is done until the end of the buffer string is found or until MAX_FIELDS are reached and returns.
```c
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
### getFieldString
Returns the value of the field requested if the field number is in range otherwise returns NULL.
```c
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
### getFieldInteger
Returns the integer value of the field if the field number is in range and the field type is numeric otherwise returns NULL
```c
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
### isCommand
This function returns true if the command matches the first field and the number of arguments is greater than or equal to the requested number of minimum arguments. 
```c
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
### strcompare
Compares if two stings are equal in length and content.
```c
bool strcompare(const char str1[], const char str2[])
{
    while(*str1 != '\0' && *str2 != '\0')
    {
        if(*str1 != *str2)
        {
            return false;
        }
        str1++;
        str2++;
    }
    if(*str1 == '\0' && *str1 == '\0')
    {
        return true;
    }
    else
    {
        return false;
    }
}
```
### intToString
Converts an integer between -32768 to 32768 to a string 
```c
void intToString(int16_t val)
{
    char str[8];
    int index = 0;
    uint16_t posVal;

    if (val < 0)
    {
        str[index++] = '-';

        if (val == -32768)
            posVal = 32768;
        else
            posVal = (uint16_t)(-val);
    }
    else
    {
        posVal = (uint16_t)val;
    }

    bool started = false;

    if (posVal >= 10000)
    {
        str[index++] = '0' + posVal / 10000;
        posVal %= 10000;
        started = true;
    }
    if (posVal >= 1000 || started)
    {
        str[index++] = '0' + posVal / 1000;
        posVal %= 1000;
        started = true;
    }
    if (posVal >= 100 || started)
    {
        str[index++] = '0' + posVal / 100;
        posVal %= 100;
        started = true;
    }
    if (posVal >= 10 || started)
    {
        str[index++] = '0' + posVal / 10;
        posVal %= 10;
        started = true;
    }

    str[index++] = '0' + posVal;
    str[index] = '\0';

    putsUart0(str);
}
```
## main.c
```c

```
## uart.c
```c

```
# wait.c
```c

```
