/*
 * File:   mainSPI.c
 * Author: USUARIO1
 *
 * Created on 15 de diciembre de 2021, 0:38
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

#define baud_9600 51// Para un valor de 2MHz

char txbuffer[200];
unsigned char dataCMD_ISR; 

unsigned int dutyOC1 = 1999; 
unsigned int dutyOC2 = 3999; 

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
unsigned int nextchar = 0; //no se usa para el oc1 oc2

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

void spi_config(void)
{
    
    //Configuración de pines SPI
    TRISCbits.TRISC0 = 1; //MISO asignado a RC0 (PIN 25), funciona como un pin de entrada por eso esta a 1
    //el resto de pines como son generados como master son salidas
    TRISCbits.TRISC1 = 0; //MOSI asignado a RC1 (PIN 26)
    TRISCbits.TRISC2 = 0; //SCK asignado a RC2C(PIN 27)
    TRISCbits.TRISC3 = 0; //CS asignado a RC3 (PIN 36)
    
    LATCbits.LATC1 = 0;
    
    //aqui se remapea (los valores que pongo son los que me ofrece el fabricante)
    RPINR20bits.SDI1R = 16; //RC0 trabajando como MISO (entrada)
    RPOR8bits.RP17R = 7; //(00111);    RC1 es MOSI (salida)
    RPOR9bits.RP18R = 8; //(01000);    RC2 es SCK (salida)
    
    //Configuración SPISTAT
    SPI1STATbits.SPIEN = 0;
    SPI1STATbits.SPISIDL = 0;
    SPI1STATbits.SPIROV = 0;
    SPI1STATbits.SPITBF = 0;
    SPI1STATbits.SPIRBF = 0;
    
    //Configuración SPICON1
    SPI1CON1bits.DISSCK = 0;
    SPI1CON1bits.DISSDO = 0;
    SPI1CON1bits.MODE16 = 0; //el spi puede hacer transferencias en una sola tanda de 16 bits y sino la establecemos se hacen transferencias de 8 bits
    SPI1CON1bits.SMP = 0;
    
    SPI1CON1bits.CKP = 0; //SPI MODE 1
    SPI1CON1bits.CKE = 0;
    
    SPI1CON1bits.SSEN = 0; //Hardware SS not used
    SPI1CON1bits.MSTEN = 1; //Master mode
    
    SPI1CON1bits.PPRE = 1; //1:1
    SPI1CON1bits.SPRE = 6; //2:1  //Fspi = Fcpu/2 = 1MHz/1Mbps
    
    //Configuración SPICON2 Framed Mode Disabled
    SPI1CON2bits.FRMEN = 0;
    SPI1CON2bits.SPIFSD = 0;
    SPI1CON2bits.FRMPOL = 0;
    SPI1CON2bits.FRMDLY = 0;
    
    //SPI Interrupciones
    IFS0bits.SPI1IF = 0;
    IFS0bits.SPI1EIF = 0;
    IEC0bits.SPI1IE = 0;
    IEC0bits.SPI1EIE = 0;
    IPC2bits.SPI1EIP = 6;
    IPC2bits.SPI1EIP = 6;
    
    LATCbits.LATC3 = 1;
    SPI1STATbits.SPIEN = 1;
    
}

//funcion que me permite tanto leer como escribir, pero para leer tenemos que escribir
unsigned char spi_write (unsigned char data) 
{
    //si no esta libre nos quedamos en el bucle while
    while(SPI1STATbits.SPITBF); //Esperamos a que el buffer de transmision este vacío
    SPI1BUF = data; //Cargamos un dato al buffer
    while(!SPI1STATbits.SPIRBF); //Esperamos a recibir el sitio de vuelta. Basicamente esperamos a que el SPI muestre durante 8 ciclos de retorno
    
    return SPI1BUF;
}

int main(void) {
    
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
    
    spi_config();
    delay_ms(10);
    
    /*Habilitar Interrupciones globales*/
    INTCON1bits.NSTDIS = 0; //Interrupt nesting enable
    SRbits.IPL = 0; //enable global interrupts
    
     while(1)
    {
        
        //secuencia de habilitacion (write enable) 
        LATCbits.LATC3 = 0;
        delay_ms(5);
        spi_write(0x06);
        
        delay_ms(5);
        LATCbits.LATC3 = 1;
        
        sprintf(txbuffer, "Habilitacion de escritura completada \r\n");
        EnviarString(txbuffer);
        delay_ms(1000);
        
        //operacion de escritura
        LATCbits.LATC3 = 0;
        delay_ms(5);
        
        spi_write(0x02); //instruccion de tipo escritura
        
        //direccion de memoria
        spi_write(0x00); //parte alta MSB
        spi_write(0x01); //parte baja LSB
        
        //escribimos el valor 5 en la direccion 0x01
        spi_write(0x05); 
        
        delay_ms(5);
        LATCbits.LATC3 = 1;
        
        sprintf(txbuffer, "Escritura completada (valor 5 en direccion 0x0001) \r\n");
        EnviarString(txbuffer);
        
        delay_ms(5000);
        
        //queremos contrastar que lo que hemos escribido en nuetra direccion de memoria coincide con 0x05
        //operacion de lectura
        LATCbits.LATC3 = 0;
        delay_ms(5);
        
        spi_write(0x03); //instruccion de tipo lectura
        
        //indicamos a donde quremos acceder
        //direccion de memoria
        spi_write(0x00); //parte alta MSB
        spi_write(0x01); //parte baja LSB
        
        dataCMD_ISR = spi_write(0x00); //si esta en modo idle se puede utilizar 0xFF
        
        delay_ms(5);
        LATCbits.LATC3 = 1;
        
        delay_ms(1000);
        sprintf(txbuffer, "Dato en memoria %02X, Adress = 0x0001 \r\n", dataCMD_ISR);
        EnviarString(txbuffer);
    }
    
    EnviarString(txbuffer);
    return 0;
}
