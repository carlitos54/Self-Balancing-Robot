#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "clock.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"

#define MAX_CHARS 80
#define MAX_FIELDS 5

typedef struct _USER_DATA
{
    char buffer[MAX_CHARS+1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
} USER_DATA;

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
