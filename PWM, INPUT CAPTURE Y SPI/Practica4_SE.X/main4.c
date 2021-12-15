/*
 * File:   main4.c
 * Author: USUARIO1
 *
 * Created on 11 de diciembre de 2021, 22:33
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
#include <stddef.h>
#include <stdbool.h>
#include <math.h>

#define baud_9600 4000000// Para un valor de 4MHz
#define LED_RED LATBbits.LATB0;
#define LED_GREEN LATBbits.LATB1;

char txbuffer[200];
char dataCMD_ISR[50]; 

unsigned int dutyOC1 = 1599; 
unsigned int dutyOC2 = 2799; 

unsigned int pulsoIC1 = 0;
unsigned int time_pulsoIC1 = 0;
unsigned int rise_pulsoIC1 = 0;

unsigned int pulsoIC2 = 0;
unsigned int time_pulsoIC2 = 0;
unsigned int rise_pulsoIC2 = 0;

double tiempo_real_IC1 = 0.00;
double tiempo_real_IC2 = 0.00;

char received_char; 
unsigned int flag = 1;

// Modulo Uart. Variables empleando ISR
char txbuffer_ISR[200];
unsigned int nextchar = 0; 


void delay_ms(unsigned long time_ms)
{
    unsigned long u;
    for(u = 0; u < time_ms * 90; u++) 
    {
        asm("nop");
    }
}

void uart_config (unsigned int baud) //esta no la he usado en IC1 IC2
{
     // Configuración de pines tx y rx
    TRISCbits.TRISC0  = 1;   // Pin de recepcion de uart establecido como entrada.
    RPINR18bits.U1RXR = 16;  // pin de recepcion rc0 trabajando con el modulo uart (RP16)
    RPOR8bits.RP17R   = 3;   // U1TX conectado con el pin RC1 (RP17)
    
    // Configuración de registro de U1MODE
    U1MODEbits.UARTEN = 0;     // Deshabilitar Uart.
    U1MODEbits.USIDL  = 0;     // Continuar operación en modo IDLE
    U1MODEbits.IREN   = 0;     // IR no usado
    U1MODEbits.RTSMD  = 1;     // Control de flujo desactivado.
    U1MODEbits.UEN    = 0;     // Solo usamos pin de Tx y pin de Rx
    U1MODEbits.WAKE   = 0;     // No quiero que la UART despierte del modo sleep
    U1MODEbits.LPBACK = 0;     // Loopback deshabilitado.
    U1MODEbits.ABAUD  = 0;     // Automedición de baudios (bps) deshabilidada
    U1MODEbits.URXINV = 0;     // En estado de reposo, el receptor mantiene un estado alto, high
    U1MODEbits.BRGH   = 1;     // Modo High-Speed
    U1MODEbits.PDSEL  = 0;     // 8 Bits de datos y paridad Nula (8N)
    U1MODEbits.STSEL  = 0;     // 1-bit de stop al final de la trama de datos.   (8N1)

    // Configuración de registro de U1STA
    U1STAbits.UTXISEL0 = 0;    // Tema interrupciones 
    U1STAbits.UTXISEL1 = 0;    // Tema interrupciones 
     
    U1STAbits.UTXINV   = 0;    // El estado en reposo del pin de transmisión es High
    U1STAbits.UTXBRK   = 0;    // No usamos trama de sincronización
    U1STAbits.UTXEN    = 1;    // El transmisor a pleno funcionamiento.
    U1STAbits.URXISEL  = 0;    // Tema interrupciones (no mirar aun)
    U1STAbits.ADDEN    = 0;    // No usamos direccionamiento.
    U1STAbits.OERR     = 0;    // Reseteamos buffer de recepción

    // Configuramos la velocidad de transmisión/recepcción de los datos
    U1BRG = baud;
      
    // Prioridades, flags e interrupciones correspondientes a la Uart
    IPC2bits.U1RXIP = 6;    // U1RX con nivel de prioridad 6 (7 es el maximo)
    IFS0bits.U1RXIF = 0;    // Reset Rx Interrupt flag
    IEC0bits.U1RXIE = 1;    // Enable Rx interrupts
      
    IPC3bits.U1TXIP = 5;    // U1TX con nivel de prioridad 6 (7 es el maximo)
    IFS0bits.U1TXIF = 0;    // Reset Tx Interrupt flag
    IEC0bits.U1TXIE = 0;    // Enable Tx interrupts
     
    U1MODEbits.UARTEN = 1;     // Uart habilitada por completo
}

void output_compare_config(void) 
{
    //Configurar pines con los que vamos a trabajar (RP:de remapeable en proteus)
    TRISCbits.TRISC2 = 0;   //Pin configurado como salida
    TRISCbits.TRISC3 = 0;   //Pin configurado como salida
    RPOR9bits.RP18R = 0x12; //Pin RC0 conectado al modulo OC1 subreg:RP16
    RPOR9bits.RP19R = 0x13; //Pin RC1 conectado al modulo OC2 subreg:RP17   
    
    /*Inicializacion modulo OC*/
    OC1CONbits.OCM = 0;  //Deshabilitar PWM
    OC1CONbits.OCTSEL = 0; //Timer 2 como base de tiempos para el OC
    OC1R = 1599;  //1 miliseg 
    OC1RS = 1599;  //1 miliseg (duty clycle 40%)
    
    OC2CONbits.OCM = 0; //Deshabilitar PWM
    OC2CONbits.OCTSEL = 0; //Timer 2 como base de tiempos para el OC
    OC2R = 2799; //1 miliseg //esto es solo de lectura tecnicamente incluso se podria eliminar
    OC2RS = 2799; //1 miliseg (duty clycle 70%)
    
    /*Configuración Timer 2*/
    T2CONbits.TON = 0; //Deshabilitamos el Timer 2
    T2CONbits.TCS = 0; //Seleccionamos el clock interno
    T2CONbits.TGATE = 0; //Gated timer mode deshabilitado
    T2CONbits.T32 = 0; //Timer 2 modo de 16-bits de resolucion
    T2CONbits.TCKPS = 0; //Prescaler 1:1
    
    /*Configurar registros de interrupcion de timer 2*/
    IPC1bits.T2IP = 5; //Prioridad de nivel 5
    IFS0bits.T2IF = 0; //Reset flag    
    IEC0bits.T2IE = 0; //Deshabilitar interrupcion
    
    PR2 = 3999; //Periodo de 1ms tiempo PRESCALER*(reg + 1)/Fbus
                //Fbus = 4MHz
    
    /*Arrancar modulos*/
    OC1CONbits.OCM = 6; //Modo PWM
    OC2CONbits.OCM = 6; //Modo PWM
    T2CONbits.TON = 1; //Iniciar Timer 2
    
}

void input_capture_config(void)
{
    //Configurar pines con los que vamos a trabajar (RP:de remapeable en proteus)
    TRISCbits.TRISC5 = 1;   //Pin configurado como entrada
    TRISCbits.TRISC6 = 1;   //Pin configurado como entrada
    RPINR7bits.IC1R = 21;   // R7 por IC1 y 21 por RP21 (remapping))
    RPINR7bits.IC2R = 22;   // R7 por IC2 y 22 por RP22 (remapping))
    
    //Configurar IC1
    IC1CONbits.ICM    = 0;
    IC1CONbits.ICSIDL = 0;  // el IC se detiene si la CPU entra en modo reposo
    IC1CONbits.ICTMR  = 0;  // Trabajamos con Timer 3
    IC1CONbits.ICI    = 0;  // salta en cada evento, es decir, cada vez que se detecta un flanco de subida o de bajada
    
    //Configurar registros de interrupcion para IC1
    IPC0bits.IC1IP = 5; //Setup IC1 interrupt priority level
    IFS0bits.IC1IF = 0; //clear IC1 Interrupt Status flag
    IEC0bits.IC1IE = 1;  // Enable IC1 interrupts
    
    //Configurar IC2
    IC2CONbits.ICM    = 0;
    IC2CONbits.ICSIDL = 0;  // el IC se detiene si la CPU entra en modo reposo
    IC2CONbits.ICTMR  = 0;  // Trabajamos con Timer 3
    IC2CONbits.ICI    = 0;  // salta en cada evento, es igual que el ICI del IC1 (001) según la documentacion
    
    //Configurar registros de interrupcion para IC2
    IPC1bits.IC2IP = 5; //Setup IC2 interrupt priority level
    IFS0bits.IC2IF = 0; // clear IC2 interrupt status flag
    IEC0bits.IC2IE = 1;  // Enable IC2 interrupt
     
    //Configuramos Timer3
    //T3CONbits.TSIDL   = 0;
    T3CONbits.TON     = 0; //Deshabilitamos el timer 3
    T3CONbits.TGATE   = 0; //gated timer mode deshabilitado
    T3CONbits.TCKPS   = 0; //Prescaler mas bajo = 1. (1:1)
    T3CONbits.TCS     = 0; //seleccionar timer interno
    //PR3 = 10000;
    
    /*Configuracion de registros de interrupcion del timer 3*/
    IPC2bits.T3IP = 5; //Prioridad de nivel 5
    IFS0bits.T3IF = 0; //reset flag
    IEC0bits.T3IE = 0; //Deshabilitamos interruociones
    
    // Activar los modulos IC1, IC2 y Timer3
    T3CONbits.TON     = 1;  //Activar Timer3
    IC1CONbits.ICM    = 3;  //Activar modulo OC1 por defecto activar el primer flanco de subida
    IC2CONbits.ICM    = 3;  //Activar modulo OC2 por defecto activar el primer flanco de subida
}

void EnviarCaracter(char c) 
{
    while(U1STAbits.UTXBF); //Mientras el buffer del puerto U1 este lleno, esperar al bucle while
    U1TXREG = c; //Si no esta lleno, se envia el byte
}

void EnviarString(char *s) 
{
    while((*s) != '\0'){
        EnviarCaracter(*(s++)); //Mientras no se haya llegado a un caracter nulo, entonces se continua imprimiendo los datos
    }
}

int main(void) //todavia no he unido ambas 
{
    //Configurar el oscilador para funcionar a 4MHz
    //Fosc = Fin * M/(N1 * N2)
    //Fosc = 8M * 40(2 * 2) = 80 MHz para un reloj de 8MHz de entrada
    //Fcy = Fosc/2
    
    PLLFBD = 2;         //M = 4
    CLKDIVbits.PLLPOST = 0; //N1 = 2
    CLKDIVbits.PLLPRE = 0;  //N2 = 2
    RCONbits.SWDTEN = 0;    //Disable watch Dog Timer
    while(OSCCONbits.LOCK != 1); //wait for PLL to lock  
    
    /*Pines analogos o digitales*/
    AD1PCFGL = 0xFFFF; //Todos los puertos son digitales
    
    /*Configuración de pines auxiliares*/
    TRISBbits.TRISB0 = 0; //Pin RB0 como salida (LED 1)
    TRISBbits.TRISB1 = 0; //Pin RB1 como salida (LED 2)
    LATBbits.LATB0 = 0; //Pin RB0 a nivel bajo por defecto (LED1 APAGADO)
    LATBbits.LATB1 = 0; //Pin RB1 a nivel bajo por defecto (LED2 APAGADO)
    
    uart_config(baud_9600); //Configurar el modulo UART
    delay_ms(10);
    
    output_compare_config(); //Configurar modulo PWM
    delay_ms(10);
    
    input_capture_config(); //configura modulo input capture //hay que mirar como poner las interrupciones
    delay_ms(10);   
    
    /*Habilitar Interrupciones globales*/
    INTCON1bits.NSTDIS = 0; //Interrupt nesting enable
    SRbits.IPL = 0; //enable global interrupts
    
    sprintf(txbuffer, "\r\n SIMULACION SISTEMAS EMPOTRADOS PRACTICA 4 \r\n");
    EnviarString(txbuffer);
    sprintf(txbuffer, " -------------------------- \r\n\r\n");
    EnviarString(txbuffer);
    delay_ms(100);
    
    //Leds
    TRISBbits.TRISB0 = 0; //SALIDA
    TRISBbits.TRISB1 = 0; //SALIDA
    //LED_RED = 0;
    //LED_GREEN = 0;
            
    while(1)
    {
        if(U1STAbits.OERR) U1STAbits.OERR = 0; //Si hay overflow en el buffer de recpción, reseteamos la UART
    
        if(flag)
        {
            tiempo_real_IC1 = 1.0*((double)time_pulsoIC1)/4000.0;
            tiempo_real_IC2 = 1.0*((double)time_pulsoIC2)/4000.0;
    
            sprintf(txbuffer, "Time_IC1: %05.3fms  Time_IC2: %1ms \r\n", tiempo_real_IC1, tiempo_real_IC2);
            EnviarString(txbuffer);
            
            //esto se hace para evitar saturar el simulador
            IFS0bits.U1RXIF = 0;
            IEC0bits.U1RXIE = 1;
            
            flag = 0;
            delay_ms(100);
        }
        
    }
    
    EnviarString(txbuffer);
    return 0;
}

//evaluar el ciclo de trabajo
void __attribute__((__interrupt__, no_auto_psv)) _IC1Interrupt(void) //este es nombre especifico y no se puede cambiar
{ 
    /*Si el pulso esta a 0 voy a volcar el dato que tenga de temporización el 'IC1BUF' y lo asignos
     * a una varibale.
     * Una vez asignada la marca de tiempos en la que estoy, lo que hago es cambiar opara que la interrupcion
     * salte con el flanco de bajada (recordando que partimos de un flanco de subida)
     * Y una vez que  tengo eso ponemos el pulso a 1, porque lo siguiente a evaluar es un cambio a flanco de subida
     *  */
    if(pulsoIC1 == 0){ 
        rise_pulsoIC1 = IC1BUF;
        IC1CONbits.ICM = 2; // Capture next falling edge
        pulsoIC1 = 1;

    }else{
        /*Una vez detectado el flanco de bajado, declaro una variable y cogemos la marca de tiempos actual y le restamos
         * el valor anterior 
         * cambiamos a flanco de subida y ponemos el puso 1 a 0 para que vuleva repetirse el if de arriba  */
        time_pulsoIC1 = IC1BUF - rise_pulsoIC1;
        IC1CONbits.ICM = 3; // Capture next rising edge
        pulsoIC1 = 0;

    }
        
    IFS0bits.IC1IF = 0;          // Reset IC1 Interrupt
}

void __attribute__((__interrupt__, no_auto_psv)) _IC2Interrupt(void)
{ 
    
    if(pulsoIC2 == 0){
        rise_pulsoIC2 = IC2BUF;
        IC2CONbits.ICM = 2; // Capture next falling edge
        pulsoIC2 = 1;

    }else{
        time_pulsoIC2 = IC2BUF - rise_pulsoIC2;
        IC2CONbits.ICM = 3; // Capture next rising edge
        pulsoIC2 = 0;
    }
        
    IFS0bits.IC2IF = 0;          // Reset IC2 Interrupt
}

