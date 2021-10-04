/*
 * File:   main_p1.c
 * Author: María González
 *         Santiago Molpeceres
 * Created on 1 de octubre de 2021, 11:56
 */

/// DSPIC33FJ32MC204 Configuration Bit Settings
// 'C' source line config statements
// FBS

#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
 

// FGS

#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)
 

// FOSCSEL

#pragma config FNOSC = PRIPLL           // Oscillator Mode (Primary Oscillator (XT, HS, EC) w/ PLL)
#pragma config IESO = ON                // Internal External Switch Over Mode (Start-up device with FRC, then automatically switch to user-selected oscillator source when ready)
 

// FOSC

#pragma config POSCMD = EC              // Primary Oscillator Source (EC Oscillator Mode)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 pin has clock out function)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow Multiple Re-configurations)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are disabled)


// FWDT

#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = ON              // Watchdog Timer Enable (Watchdog timer always enabled)

 
// FPOR

#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)
#pragma config LPOL = ON                // Motor Control PWM Low Side Polarity bit (PWM module low side output pins have active-high output polarity)
#pragma config HPOL = ON                // Motor Control PWM High Side Polarity bit (PWM module high side output pins have active-high output polarity)
#pragma config PWMPIN = ON              // Motor Control PWM Module Pin Mode bit (PWM module pins controlled by PORT register at device Reset)

 
// FICD

#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)


// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

 
#include <xc.h>

#define LED_ROJO LATBbits.LATB3
#define LED_VERDE LATAbits.LATA0
#define LED_VERDE2 LATAbits.LATA1
#define LED_state 0

//Para contar tiempo
void delay_ms (unsigned long time_ms)
{
    unsigned long u=0;
    while(u<time_ms*450){
        asm("NOP");
        u++;
    }
}

void enciende_Apaga_Rojo()
{

    LED_ROJO = !PORTBbits.RB3;
}

void enciende_Apaga_Verde_Rojo()
{

    LED_ROJO = !PORTBbits.RB3;
    LED_VERDE2 = !PORTAbits.RA1;
}

//4MHz ---> 1Hz (1Hz = 0.000001MHz)
//Frecuencia de la CPU 4MHz a un parpadeo de 1Hz
int main(void) {


   

    PLLFBD = 2;                           //M  = 4
    CLKDIVbits.PLLPRE = 0;               //N1 = 2
    CLKDIVbits.PLLPOST = 0;              //N2 = 2
    while(OSCCONbits.LOCK != 1); //Wait for PLL to lock

    AD1PCFGL = 0xFFFF;

    //Control de direccionalidad de los pines de la practica 1

    TRISBbits.TRISB3 = 0; //Pin RB3 del puerto B como salida (output) D1
    TRISAbits.TRISA0 = 0; //Pin RA0 del puerto A como salida (output) D2
    TRISAbits.TRISA1 = 0; //Pin RA1 del puerto A como salida (output) D3

   
    TRISBbits.TRISB4 = 1; //Pin RB4 del puerto B como entrada (input)
    TRISBbits.TRISB7 = 1; //Pin RB7 del puerto B como entrada (input)

    //LED_ROJO          //Pin RB3 (D1)
    //LED_VERDE         //Pin RA0 (D2)
    //LED_VERDE2        //Pin RA1 (D2)

     while(1)
    {
        //EJERCICIO 1

         enciende_Apaga_Rojo();

         delay_ms(500);

       
        //EJERCICIO 2
        if(PORTBbits.RB4 == 0)
        {
           LED_VERDE = 1;//ON LED_VERDE (D2)
           LED_VERDE2 = 1;//ON LED_VERDE2 (D3)
        } else {
           LED_VERDE = 0;//OFF LED_VERDE (D2)
           LED_VERDE2 = 0;//OFF LED_VERDE2 (D3)
        }

       //EJERCICIO 3 
         if(PORTBbits.RB7==0){
             enciende_Apaga_Verde_Rojo();
             delay_ms(500);
             enciende_Apaga_Rojo();
             delay_ms(500);
             enciende_Apaga_Verde_Rojo();
             delay_ms(500);
             enciende_Apaga_Rojo();
             delay_ms(500);
         }else{
             enciende_Apaga_Rojo();
             delay_ms(500);
         }
    }
    return 0;
}




