/*
 * File:   main3.c
 * Author: USUARIO1
 *
 * Created on 28 de noviembre de 2021, 20:41
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
#include <string.h>
#include <stdlib.h>
#include <stddef.h> //defines NULL
#include <stdbool.h> //defines true and false
#include <math.h>

//#define CPU_40MHz
#ifdef CPU_40MHz
#define baud_9600 1041
#else
#define baud_9600 51 //FCPU = 2MHz
#endif

char dataCMD_ISR[50]; //data received from the UART

//Módulo UART. Variables empleando ISR
char txbuffer_ISR[200]; //buffer de transmisión
unsigned int nextchar = 0; //indice del siguiente caracter a enviar

volatile unsigned int data_count = 0;
volatile unsigned int comando_detectado = 0; 

unsigned char dummy;

//listado de comandos => vienen explicados en la practica 3
const char cmd1[] = {"set ledgreen 1"}; //Comando para encender el led verde
const char cmd2[] = {"set ledgreen 0"}; //Comando para apagar el led verde
const char cmd3[] = {"set ledred 1"}; //Comando para encender el led rojo
const char cmd4[] = {"set ledred 0"}; //Comando para apagar el led rojo
const char cmd5[] = {"print MIC2_data"}; //Comando para imprimir el valor del ADC
const char cmd6[] = {"stop MIC2_data"}; //Comando para parar de imprimir el valor del ADC

//FUNCIONES
//void delay_ms(unsigned long time_ms);
//void uart_config(unsigned int baud);
//void __atribute__((interrupt, no_auto_psv)) _T1Interrupt(void);
//void __atribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void);
//void __atribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void);

void timer1_config(void) {
    T1CONbits.TON = 0; //stop timer
    T1CONbits.TCS = 0; //internal clock
    T1CONbits.TSYNC = 0; //synchronize external clock input  
    T1CONbits.TCKPS = 3; //prescaler 256
    T1CONbits.TSIDL = 0; //continue in idle mode

    IPC0bits.T1IP = 5; //priority 5
    IFS0bits.T1IF = 0; //clear interrupt flag
    IEC0bits.T1IE = 0; //disable interrupt

    PR1 = 0x0005; //period
    TMR1 = 0; //reset timer

    T1CONbits.TON = 1; //Inicializar el temporizador
}

//Funcion para configurar el tiempo en ms
void delay_ms(unsigned long time_ms) 
{
    unsigned long i;
    for(i=0;i<time_ms * 90;i++) //Calculo aproximado para una CPU a 2MHz
    {
        asm("nop");
    }
}

//Funcion para configurar el UART
void uart_config(unsigned int baud)
{
    //Configuración de pines tx y rx
    TRISCbits.TRISC0 = 1;      //Pin de recepción de UART establecido como entrada
    RPINR18bits.U1RXR = 16;    //Pin de recepción RC0 trabajando con el módulo UART (RP16)
    RPOR8bits.RP17R = 3;       //U1TX conectado con el pin RC1 (RP17)

    //Configuración de registro de U1MODE
    U1MODEbits.UARTEN = 0;     //UART deshabilitado
    U1MODEbits.USIDL = 0;      //Operaciones en modo de reposo
    U1MODEbits.IREN = 0;       //IR no usado
    U1MODEbits.RTSMD = 1;      //Control de flujo por software (modo de transmisión) -> desactivado
    U1MODEbits.UEN = 0;        //Solo usamos el pin de Tx y Rx
    U1MODEbits.WAKE = 0;       //No quiero que la UART despierte del modo sleep
    U1MODEbits.LPBACK = 0;     //Loopback deshabilitado
    U1MODEbits.ABAUD = 0;      //Automedición de baudios (bps) deshabilitada
    U1MODEbits.URXINV = 0;      //En estado de reposo, el receptor mantiene un estado alto, high
    U1MODEbits.BRGH = 1;       //Modo High Speed
    U1MODEbits.PDSEL = 0;      //8 bits de datos y paridad nula (8N)
    U1MODEbits.STSEL = 0;      //1 bit de stop al final de la trama de datos (8N1)

    //Configuración de registro de U1STA
    U1STAbits.UTXISEL0 = 0;    //Tema insterrupciones
    U1STAbits.UTXISEL1 = 0;    //Tema insterrupciones 

    U1STAbits.UTXINV = 0;      //El estado en reposo del pin de transmisión es High
    U1STAbits.UTXBRK = 0;      //No usamos trama de sincronización
    U1STAbits.UTXEN = 1;       //Habilitamos la transmisión
    U1STAbits.URXISEL = 0;     //Tema interrupciones (no hay que mirarlo aun)
    U1STAbits.ADDEN = 0;       //No usamos direccionamiento
    //U1STAbits.RIDLE = 0;
    U1STAbits.OERR = 0;        //Reseteamos buffer de recepción

    //Configuramos la velocidad de trasnmisión/recepción de los datos
    U1BRG = baud;

    //Prioridades, flags e interrupciones correspondientes a la UART
    //RX
    IPC2bits.U1RXIP = 6;      //U1RX con nivel de prioridad 6 (7 es el máximo)
    IFS0bits.U1RXIF = 0;      //Resets Rx interrupción flag
    IEC0bits.U1RXIE = 1;      //Enable Rx interrupción

    //Tx
    IPC3bits.U1TXIP = 5;      //U1TX con nivel de prioridad 5 (6 es el máximo)
    IFS0bits.U1TXIF = 0;      //Resets Tx interrupción flag
    IEC0bits.U1TXIE = 1;      //Enable Tx interrupción

    U1MODEbits.UARTEN = 1;    //Habilitamos la UART
}



int main(void) {

    #ifdef CPU_40MHz
    //Configurar el oscilador para hacer funsionar la CPU a 40MHz a partir de un reloj de entrada de 8MHz
    //Fosc = Fin * M/(N1*N2), Fcy = Fosc/2
    //Fosc = 8M * 40/(2*2) = 80MHz para un reloj de 8MHz de entrada
    //Fcy = Fosc/2 = 80/2 = 40MHz

    PLLFBD = 0;              //M = 2
    CLKDIVbits.PLLPOST = 0;  //N1 = 2
    CLKDIVbits.PLLPRE = 0;   //N2 = 2
    while(OSCCONbits.LOCK != 1); //Esperamos a que el oscilador se haya configurado correctamente

    #else
    //Configurar el oscilador para hacer funsionar la CPU a 2MHz a partir de un reloj de entrada de 8MHz
    //Fosc = Fin * M/(N1*N2), Fcy = Fosc/2
    //Fosc = 8M * 2/(2*2) = 4MHz para un reloj de 8MHz de entrada
    //Fcy = Fosc/2 = 4/2 = 40MHz

    PLLFBD = 0;              //M = 2
    CLKDIVbits.PLLPOST = 0;  //N1 = 2
    CLKDIVbits.PLLPRE = 0;   //N2 = 2
    while(OSCCONbits.LOCK != 1); //Esperamos a que el oscilador se haya configurado correctamente
    #endif

    AD1PCFGL = 0xFFFF; //Todos los pines como digitales
    TRISAbits.TRISA0 = 0;
    TRISBbits.TRISB3 = 0;
    LATAbits.LATA0 = 0;
    LATBbits.LATB3 = 0;

    uart_config(baud_9600);
    PR1 = 24999; //Tiempo de interrupcion de timer1 (1ms)

    while(1) {
        if(comando_detectado) {
            if(!strcmp(((const char*) dataCMD_ISR), cmd1)) {
                memset(txbuffer_ISR, '\0', sizeof(txbuffer_ISR)); //Resetear buffer con NULL
                sprintf(txbuffer_ISR, "%c%c%c", 0x50, 0x01, 0xAA); //Enviamos un mensaje de respuesta
                nextchar = 0; //Resetear contador de caracteres

                if(U1STAbits.UTXBF) {
                    IFS0bits.U1TXIF = 0; //Reseteo el flag de transmisión ISR
                }

                asm("nop");
                IEC0bits.U1TXIE = 1; //Iniciamos una nueva transmisión
            }

            if(!strcmp(((const char*) dataCMD_ISR), cmd2)) {
                memset(txbuffer_ISR, '\0', sizeof(txbuffer_ISR)); //Resetear buffer con NULL
                sprintf(txbuffer_ISR, "%c%c%c", 0x50, 0x00, 0xAA); //Enviamos un mensaje de respuesta
                nextchar = 0; //Resetear contador de caracteres

                if(U1STAbits.UTXBF) {
                    IFS0bits.U1TXIF = 0; //Reseteo el flag de transmisión ISR
                }

                asm("nop");
                IEC0bits.U1TXIE = 1; //Iniciamos una nueva transmisión
            }

            if(!strcmp(((const char*) dataCMD_ISR), cmd3)) {
                memset(txbuffer_ISR, '\0', sizeof(txbuffer_ISR)); //Resetear buffer con NULL
                sprintf(txbuffer_ISR, "%c%c%c", 0x50, 0x01, 0xAA); //Enviamos un mensaje de respuesta
                nextchar = 0; //Resetear contador de caracteres

                if(U1STAbits.UTXBF) {
                    IFS0bits.U1TXIF = 0; //Reseteo el flag de transmisión ISR
                }

                asm("nop");
                IEC0bits.U1TXIE = 1; //Iniciamos una nueva transmisión
            }

            if(!strcmp(((const char*) dataCMD_ISR), cmd4)) {
                memset(txbuffer_ISR, '\0', sizeof(txbuffer_ISR)); //Resetear buffer con NULL
                sprintf(txbuffer_ISR, "%c%c%c", 0x50, 0x00, 0xAA); //Enviamos un mensaje de respuesta
                nextchar = 0; //Resetear contador de caracteres

                if(U1STAbits.UTXBF) {
                    IFS0bits.U1TXIF = 0; //Reseteo el flag de transmisión ISR
                }

                asm("nop");
                IEC0bits.U1TXIE = 1; //Iniciamos una nueva transmisión
            }

            if(!strcmp(((const char*) dataCMD_ISR), cmd5)) {
                memset(txbuffer_ISR, '\0', sizeof(txbuffer_ISR)); //Resetear buffer con NULL
                sprintf(txbuffer_ISR, "%c%c%c", 0x50, 0x01, 0xAA); //Enviamos un mensaje de respuesta
                nextchar = 0; //Resetear contador de caracteres

                if(U1STAbits.UTXBF) {
                    IFS0bits.U1TXIF = 0; //Reseteo el flag de transmisión ISR
                }

                asm("nop");
                IEC0bits.U1TXIE = 1; //Iniciamos una nueva transmisión
            }

            if(!strcmp(((const char*) dataCMD_ISR), cmd6)) {
                memset(txbuffer_ISR, '\0', sizeof(txbuffer_ISR)); //Resetear buffer con NULL
                sprintf(txbuffer_ISR, "%c%c%c", 0x50, 0x00, 0xAA); //Enviamos un mensaje de respuesta
                nextchar = 0; //Resetear contador de caracteres

                if(U1STAbits.UTXBF) {
                    IFS0bits.U1TXIF = 0; //Reseteo el flag de transmisión ISR
                }

                asm("nop");
                IEC0bits.U1TXIE = 1; //Iniciamos una nueva transmisión
            } else {
                //creo que tengo que añadir algo
            }

            memset(dataCMD_ISR, '\0', sizeof(dataCMD_ISR)); //Resetear buffer con NULL
            data_count = 0;
            comando_detectado = 0;

        } else {
            //creo que tengo que añadir algo
        }
        delay_ms(100);
    }
    return 0;
}


//Funcion para configurar el timer 1
void __atribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void) { 
    IFS0bits.T1IF = 0; //Reset de la interrupción timer1
}

//Funcion para configurar la interrupcion del UART
void __atribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void) {
    if(comando_detectado == 0) {
        dataCMD_ISR[data_count] = U1RXREG; //Obtenemos el caracter recibido en el buffer de recepción
        data_count++;

        if((dataCMD_ISR[data_count - 1] == 13)) { //Retorno de carro-Intro detectado
            dataCMD_ISR[data_count - 1] = '\0'; //Eliminamos el retorno de carro del stram recibido
            comando_detectado = 1; //Cambiamos el valor de la bandera
            data_count = 0; //Reseteamos el contador de datos
        }
    }
    IFS0bits.U1RXIF = 0; //Reset Rx Interrupción
}

//Funcion para configurar la interrupcion del UART
void __atribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void) {
    IEC0bits.U1TXIE = 0; //Deshabilitamos la interrupción de transmisión Tx

    if(!U1STAbits.UTXBF) { //Mientras el buffer de transmisión no este lleno por completo, se continua cargando el buffer con más datos
        U1TXREG = txbuffer_ISR[nextchar++]; //Cargamos el buffer con un nuevo dato
        asm("nop");

        if(U1STAbits.UTXBF) { //Si el buffer de transmision se completó con el último dato incorporado al buffer, procedemos a resetear el flag (Este método se usa para UTXISEL = 0)
            IFS0bits.U1TXIF = 0; //Clear UART1 Tx interrupt flag
        }
    }else {
        IFS0bits.U1TXIF = 0; //Clear UART1 Tx interrupt flag
    }

    if(!(nextchar == strlen(txbuffer_ISR))) { //Si se ha finalizado la trasnmisión de todos los caracteres --> Deshabilitar interrupción y activar flags
        //strlen: cuenta el numero de caracteres hasta encontrar el Null
        IEC0bits.U1TXIE = 1; //Enable UART1 Tx Interrupt
    }
}
