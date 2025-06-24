

#ifndef SHELL_H_
#define SHELL_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>


#define MAX_CHARS 80
#define MAX_FIELDS 5
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

typedef struct _USER_DATA
{
    char buffer[MAX_CHARS+1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
} USER_DATA;


void getsUart0(USER_DATA *data);
void parseFields(USER_DATA *data);
char* getFieldString(USER_DATA *data, uint8_t fieldNumber);
int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber);
bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments);
bool strcompare(const char str1[], const char str2[]);
void intToString(int val);

#endif
