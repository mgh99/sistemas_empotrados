/*
 * File:   main_principal.c
 * Author: María González Herrero & Santiago Molpeceres Díaz
 *
 * Created on 24 de octubre de 2021, 17:42
 */

// DSPIC33FJ32MC204 Configuration Bit Settings

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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>

#define baud_9600 1041 // Es este valor porque sale de la fórmula de UxBRG 40 * 106 / 4 * 1960
#define LED_RED  LATBbits.LATB3 //D1  RB3
#define LED_GREEN LATAbits.LATA0

void delay_ms(unsigned long time_ms) {
    unsigned long u;
    for (u = 0; u < time_ms * 650; u++) // Calculo aproximado para una CPU a 40MHz generará un delay de 0.5s a 500ml ME:+-5
    {
        asm("NOP");
    }
}

void uart_config(unsigned int baud) {
    //Asignar módulo UART a un puerto/pines de comunicacion
    TRISCbits.TRISC0 = 1; // Pin RCO como puerto de entrada (digital)
    RPINR18bits.U1RXR = 16; // pin RC0 al puerto de recepcion uart 1   RP16
    RPOR8bits.RP17R = 3; // Pin RC1 conectado al pin de transmision de la uart1

    //Configuración registro U1MODE
    U1MODEbits.UARTEN = 0; // Al inicio está deshabilitada la UART
    U1MODEbits.USIDL = 0; // Continua la operación aunque se encuentre en el modo IDLE
    U1MODEbits.IREN = 0; // IR no se usa
    U1MODEbits.RTSMD = 1; // Control de flujo descativado
    U1MODEbits.UEN = 0; // Solo usamo pin TX (transmisión) y Rx (recepción) para nuestro proyecto
    U1MODEbits.WAKE = 0; // Para que la UART no se "despierte" del modo sleep
    U1MODEbits.LPBACK = 0; // Loopback deshabilitado
    U1MODEbits.ABAUD = 0; // Auto baud rate deshabilitado
    U1MODEbits.URXINV = 0; // El estado en reposo (IDLE) es un 1(alto)
    U1MODEbits.BRGH = 1; // Modo high-speed
    U1MODEbits.PDSEL = 0; // 8 bits de datos y paridad nula (8N)
    U1MODEbits.STSEL = 0; // 1-bit de stop al final de la trama de datos(8N1)

    //Configuración registro U1STA
    U1STAbits.URXISEL0 = 0;
    U1STAbits.URXISEL1 = 0;
    U1STAbits.URXISEL = 0; // Tema interrumpciones. No mirar aun
    U1STAbits.ADDEN = 0; // No usamos direccionamiento
    U1STAbits.UTXBRK = 0; // No usar trama de sincronizacion
    U1STAbits.UTXINV = 0; // Estado de reposo del pin de transmisión es por defecto 1 _> High
    U1STAbits.OERR = 0; // Buffer de recepcion esta vacio y no hay problema de overflow
    U1STAbits.PERR = 0;
    U1STAbits.FERR = 0;
    U1STAbits.UTXEN = 1; //El transmisor está en pleno funcionamiento

    //Configuramos la velocidad de transmisión/recepción de los datos
    U1BRG = baud;

    U1MODEbits.UARTEN = 1; // UART habilitada
}

void imprime_cadena(char cadena[]) {
    int i = 0;
    char* j = &cadena[0];
    while (U1STAbits.UTXBF == 0) {
        U1TXREG = (char) (*j);
        i++;
        j++;
    }
}

void parpadeo() {
    LED_GREEN = !PORTAbits.RA0;
}

int main(void) {
    // configurada a trabajar con 40Mhz
    //Fosc = Fon * (M1/(N1*N2))
    //fOSC = 8Mhz * (40/(2*2))

    PLLFBD = 38; // M = 40
    CLKDIVbits.PLLPOST = 0; // N1 = 2
    CLKDIVbits.PLLPRE = 0; // N2 = 2;
    while (OSCCONbits.LOCK != 1); // Wait for PLL to LOCK

    AD1PCFGL = 0XFFFF; // Todos los pines configurados en digital

    TRISBbits.TRISB3 = 0; // Configuramos el pin RB3 del puerto B como salida (D1)
    TRISAbits.TRISA0 = 0; // Configuramos el pin RA0 del puerto A como salida (D2)
    TRISAbits.TRISA1 = 0; // configuramos el pin RA1 del puerto A como salida (D3)
    delay_ms(10); // Hacemos una pequeña espera para asegurar que se ha hecho la configuración antes de continuar

    LATBbits.LATB3 = 0; // Estado 'low', por defecto, en el pin RB3 (D1)
    LATAbits.LATA0 = 0; // Estado 'low', por defecto, en el pin RA0 (D2)
    LATAbits.LATA1 = 0; // estado 'low' por defecto, en el pin de RA1 (D3)
    delay_ms(10); // Hacemos una pequeña espera para asegurar que se ha hecho la configuración antes de continuar
    uart_config(baud_9600);

    // Variables necesarias
    bool h_pulsada = 0;
    char ReceivedChar;
    int contador = 0;
    char nombre[] = "Maria";

    imprime_cadena(nombre);

    while (1) {
        // Ejercicio 1
        char str[100]; // no es muy optimo, llegará un punto que deje de contar por que no entra el numero
        sprintf(str, "%d", contador);
        imprime_cadena(str);
        contador++;
        
        if (h_pulsada) {
            //he pulsado
            parpadeo();
            delay_ms(250);
        } else {
            delay_ms(500); // genera aproximadamente un delay de 0.5 sengundos s   
        }

        /* mira si ha habido un error*/
        if (U1STAbits.FERR == 1) {
            continue;
        }
        /* soluciona el overrrun */
        if (U1STAbits.OERR == 1) {
            U1STAbits.OERR = 0;
            continue;
        }
        /* obtiene el dato */
        if (U1STAbits.URXDA) {
            ReceivedChar = (char) U1RXREG;
            if (ReceivedChar == 'e' || ReceivedChar == 'E') {
                LATBbits.LATB3 = 1;
            } else if (ReceivedChar == 'a' || ReceivedChar == 'A') {
                LATBbits.LATB3 = 0;
            } else if (ReceivedChar == ' ') {
                contador = 0;
            } else if (ReceivedChar == 'h' || ReceivedChar == 'H') {
                h_pulsada = !h_pulsada;
                if (h_pulsada == 0) {
                    LED_GREEN = 0;
                }
            }
        }
        if (h_pulsada) {
            parpadeo();
            delay_ms(250);
        }
    }
    return 0;
}