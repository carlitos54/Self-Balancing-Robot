
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "wait.h"
#include "clock.h"
#include "uart0.h"
#include "shell.h"
#include "tm4c123gh6pm.h"

#define RED_LED_MASK 2
#define GREEN_LED_MASK 8
#define IN1_MASK 1                  //Port B0
#define IN2_MASK 2                  //Port B1
#define IN3_MASK 16                 //Port E4
#define IN4_MASK 32                 //Port E5
#define ENA_MASK 16                 //Port C4 M0PWM6
#define ENB_MASK 32                 //Port C5 M0PWM7
#define SCL_MASK 4                  //PB2 I2C0
#define SDA_MASK 8                  //PB3 I2C0
#define ICM_ADDR 0x69

int16_t pitch = 0;
int32_t gyro_offset = 0;
int Kp = 5;
int Kd = 2;
bool balance = 0;

typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
} IMU_Data;

IMU_Data imu_data;

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

void initTimer1A()
{
    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;        // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;  // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD; // configure for periodic mode (count down)
    TIMER1_TAILR_R = 400000;                // set load value to 40e6 for 1Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;        // turn-on interrupts for timeout in timer module
    TIMER1_CTL_R |= TIMER_CTL_TAEN;         // turn-on timer
    NVIC_EN0_R = 1 << (INT_TIMER1A-16);     // turn-on interrupt 37 (TIMER1A) in NVIC
}

void initI2C0()
{
    GPIO_PORTB_DEN_R |= SCL_MASK | SDA_MASK;            //Digital Enable
    GPIO_PORTB_AFSEL_R |= SCL_MASK | SDA_MASK;          //alternate function
    GPIO_PORTB_ODR_R |= SDA_MASK;                       //open drain
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB2_I2C0SCL | GPIO_PCTL_PB3_I2C0SDA;
    I2C0_MCR_R = I2C_MCR_MFE;                           //Master function enabled
    I2C0_MTPR_R = 19;                                   //I2C clock load
    I2C0_MSA_R = (ICM_ADDR << 1);                       //master slave address set in register
}

uint8_t RWByte(uint8_t reg)
{
    uint8_t i = 0;
    for(i = 0; i < 3; i++)                      //attempts 3 times
    {
        //send register address
        I2C0_MSA_R = (ICM_ADDR << 1) | 0;       // loads slave addr with write bit
        I2C0_MDR_R = reg;
        I2C0_MCS_R = I2C_MCS_START | I2C_MCS_RUN | I2C_MCS_STOP;
        while (I2C0_MCS_R & I2C_MCS_BUSY);
        //read from that register
        I2C0_MSA_R = (ICM_ADDR << 1) | 1;       // Read mode
        I2C0_MCS_R = I2C_MCS_START | I2C_MCS_RUN | I2C_MCS_STOP;
        while (I2C0_MCS_R & I2C_MCS_BUSY);
        return I2C0_MDR_R;
    }
    return 0xFF;                                //error
}

void writeReg(uint8_t reg, uint8_t data)
{
    I2C0_MSA_R = (ICM_ADDR << 1) | 0;       //write
    I2C0_MDR_R = reg;                       //destination reg
    I2C0_MCS_R = I2C_MCS_START | I2C_MCS_RUN;
    while (I2C0_MCS_R & I2C_MCS_BUSY);

    I2C0_MDR_R = data;                      //data to write to reg
    I2C0_MCS_R = I2C_MCS_RUN | I2C_MCS_STOP;//write until finished
    while (I2C0_MCS_R & I2C_MCS_BUSY);
}

void printIMUValues(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz)
{
    putsUart0("ACCEL:\n");
    putsUart0("   X = ");
    intToString(ax);
    putsUart0("\n");

    putsUart0("   Y = ");
    intToString(ay);
    putsUart0("\n");

    putsUart0("   Z = ");
    intToString(az);
    putsUart0("\n");

    putsUart0("GYRO:\n");
    putsUart0("   X = ");
    intToString(gx);
    putsUart0("\n");

    putsUart0("   Y = ");
    intToString(gy);
    putsUart0("\n");

    putsUart0("   Z = ");
    intToString(gz);
    putsUart0("\n");
}

int16_t readWord(uint8_t regH, uint8_t regL)
{
    uint8_t high = RWByte(regH);
    uint8_t low  = RWByte(regL);
    return (int16_t)((high << 8) | low);
}

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

void updatePitch(int16_t accel_deg, int16_t gyro_raw)
{
    gyro_raw -= gyro_offset;
    uint8_t alpha = 92;         //trust gyro 90%
    uint16_t dt = 10;        //loop period 10ms
    int16_t gyro_s = 65;   //ICM gyro LSB/deg/s    Page 11

    int32_t gyro_scaled = ((int32_t)gyro_raw * 100) / gyro_s;   //converts to deg/s
    int16_t delta = (gyro_scaled * dt) / 10;
    int16_t gyro_pitch = pitch + delta;
    pitch = ((gyro_pitch * alpha) + (accel_deg * (100 - alpha))) / 100;
}



void readIMU(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{
    *ax = readWord(0x2D, 0x2E);                 //Page 32 of Datasheet to read high and low values
    *ay = readWord(0x2F, 0x30);
    *az = readWord(0x31, 0x32);
    *gx = readWord(0x33, 0x34);
    *gy = readWord(0x35, 0x36);
    *gz = readWord(0x37, 0x38);
}

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

void calibrateGyro()
{
    int32_t sum = 0;
    int samps = 1000;
    int i = 0;
    for(i = 0; i < samps; i++)
    {
        sum += readWord(0x35, 0x36);
        waitMicrosecond(200);
    }
    gyro_offset = sum/samps;
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    USER_DATA data;
    initSystemClockTo40Mhz();
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1 | SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R4 | SYSCTL_RCGCGPIO_R5;        //Enable Port B, C, E, F clock
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;          //Enable PWM0 Clock
    SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;          //Enable I2C0 clock
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;      //Enable Timer1A clock
    _delay_cycles(3);
    GPIO_PORTF_DIR_R |= GREEN_LED_MASK |RED_LED_MASK;
    GPIO_PORTF_DEN_R |= GREEN_LED_MASK |RED_LED_MASK;
    uint16_t pwmval = 19000;
    //uint8_t whoami = 0;
    initUart0();
    initPWM();
    initTimer1A();
    initI2C0();
    RWByte(0x00);                                   //Wake IMU
    writeReg(0x06, 0x01);                           //write to whoami
    waitMicrosecond(200000);                        //time for sensors to stablize
    writeReg(0x1B, 0x08);
    calibrateGyro();
    while(true)
    {
        bool validCommand = false;
        getsUart0(&data);
        parseFields(&data);
        if (isCommand(&data, "startup", 0))
        {
            validCommand = true;
            balance = 0;
            GPIO_PORTB_DATA_R |= IN1_MASK;      //B0  Forward
            GPIO_PORTB_DATA_R &= ~IN2_MASK;     //B1  Backward
            GPIO_PORTE_DATA_R |= IN3_MASK;      // Set PE4 HIGH Forward
            GPIO_PORTE_DATA_R &= ~IN4_MASK;     // Set PE5 LOW Backward
            PWM0_3_CMPA_R = pwmval;             // Motor A (ENA)
            PWM0_3_CMPB_R = pwmval;             // Motor B (ENB)
            waitMicrosecond(3000000);
            PWM0_3_CMPA_R = 0;                  // Motor A (ENA)
            PWM0_3_CMPB_R = 0;                  // Motor B (ENB)
            waitMicrosecond(1000);
            GPIO_PORTB_DATA_R |= IN2_MASK;
            GPIO_PORTB_DATA_R &= ~IN1_MASK;
            GPIO_PORTE_DATA_R |= IN4_MASK;
            GPIO_PORTE_DATA_R &= ~IN3_MASK;
            PWM0_3_CMPA_R = pwmval;             // Motor A (ENA)
            PWM0_3_CMPB_R = pwmval;             // Motor B (ENB)
            waitMicrosecond(3000000);
            PWM0_3_CMPA_R = 0;                  // Motor A (ENA)
            PWM0_3_CMPB_R = 0;                  // Motor B (ENB)
        }
        else if (isCommand(&data, "balance", 0))
        {
            balance = 1;
            validCommand = true;
        }
        else if (isCommand(&data, "stop", 0))
        {
            balance = 0;
            validCommand = true;
            PWM0_3_CMPA_R = 0;  // Motor A (ENA)
            PWM0_3_CMPB_R = 0;  // Motor B (ENB)
        }
        else if (isCommand(&data, "angle", 0))              //prints filtered angle and raw accel angle
        {
            balance = 0;
            validCommand = true;
            printIMUValues(imu_data.ax, imu_data.ay, imu_data.az, imu_data.gx, imu_data.gy, imu_data.gz);
            int16_t raw_pitch = computePitch(imu_data.ax, imu_data.az);
            putsUart0("Pitch= ");
            intToString(computePitch(imu_data.ax, imu_data.az));
            putsUart0(" deg\n");
        }
        else if(!validCommand)
        {
            putsUart0("Invalid command\n");
        }
    }


}


