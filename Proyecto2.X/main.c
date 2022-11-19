/*
 * File:   main.c
 * Author: Byron Barrientos
 *
 * Created on 8 de noviembre de 2022, 9:00 PM
 */
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (RCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 500000
#define V_TMR0 100

//******************************************************************************
// Variables
//******************************************************************************



//******************************************************************************
// Prototipos de Funciones
//******************************************************************************

void setup(void);
void setupADC(void);
void setupPWM(void);

//******************************************************************************
// Interrupciones
//******************************************************************************



//******************************************************************************
// Código Principal
//******************************************************************************

void main(void) 
{    
    setup();
    
    while(1)
    {
        
    }
    return;
}

//******************************************************************************
// Funciones
//******************************************************************************

void setup (void)
{
    ANSELH = 0;
    
    TRISA = 0;              //Configuración del PORTA como output
    TRISB = 0b00011111;     //Configuración del PORTB como output/input
    TRISC = 0;              //Configuración del PORTC como output
    TRISD = 0;              //Configuración del PORTD como output
    
    PORTA = 0;              //Limpiamos el PORTA
    PORTB = 0;              //Limpiamos el PORTB
    PORTC = 0;              //Limpiamos el PORTC
    PORTD = 0;              //Limpiamos el PORTD
    
    OPTION_REGbits.nRBPU = 0;   // Habilitamos los pull-ups del PORTB
    IOCB = 0b01111111;    
    
    // Configuración del Oscilador Interno a 500KHz
    
    OSCCONbits.IRCF = 0b011 ;   // Selección de los 500KHz
    OSCCONbits.SCS = 1;         // Selección del Oscilador Interno
    
    // Configuración del TMR0 y su Interrupción
    
    OPTION_REGbits.T0CS = 0;        // Fosc/4
    OPTION_REGbits.PSA = 0;         // Prescaler para el TMR0
    OPTION_REGbits.PS = 0b011;      // Prescaler 1:16
    
    TMR0 = V_TMR0;                  // Asignamos valor al TMR0
    
    INTCONbits.TMR0IE = 1;  //Habilitamos la interrupción del TMR0
    INTCONbits.T0IF = 0;    //Bajamos la bandera de interrupción del TMR0
    
    // Interrupciones
    
    INTCONbits.GIE = 1;     //Habilitamos las interrupciones globales (GIE)
    INTCONbits.PEIE = 1;    //Habilitamos las interrupción del PEIE
    INTCONbits.RBIF = 1;    //Habilitamos las interrupciones del PORTB (RBIF)
    INTCONbits.RBIE = 0;    //Bajamos la bandera de interrupción del PORTB (RBIE) 
}

void setupADC(void)
{    
    //Paso 1: Selección del puerto de entrada
    
    TRISAbits.TRISA0 = 1;       //Configuración del RA0 como input
    ANSELbits.ANS0 = 1;         //Configuración del pin RA0 como análogo (AN0)
    
    TRISAbits.TRISA1 = 1;       //Configuración del RA1 como input
    ANSELbits.ANS1 = 1;         //Configuración del pin RA1 como análogo (AN1)
    
    TRISAbits.TRISA2 = 1;       //Configuración del RA2 como input
    ANSELbits.ANS2 = 1;         //Configuración del pin RA2 como análogo (AN2)
    
    TRISAbits.TRISA3 = 1;       //Configuración del RA3 como input
    ANSELbits.ANS3 = 1;         //Configuración del pin RA3 como análogo (AN3)
    
    //Paso 2: Configuración del módulo ADC
    
    ADCON0bits.ADCS0 = 1;
    ADCON0bits.ADCS1 = 0;       //Fosc/8
    
    ADCON1bits.VCFG0 = 0;       //VDD como voltaje de referencia -
    ADCON1bits.VCFG1 = 0;       //VSS como voltaje de referencia +
    
    ADCON0bits.CHS0 = 0;
    ADCON0bits.CHS1 = 0;
    ADCON0bits.CHS2 = 0;
    ADCON0bits.CHS3 = 0;        //Selección del canal análogo AN0 (Default)
    
    ADCON1bits.ADFM = 0;        //Justificado hacia la izquierda
    
    ADCON0bits.ADON = 1;        //Habilitamos el ADC
    
    __delay_us(100);            //Delay para adquirir la lectura
}

void setupPWM(void)
{    
    // Paso 1
    
    TRISCbits.TRISC2 = 1;           //Configuración del RC1 como input (CCP1)
    TRISCbits.TRISC1 = 1;           //Configuración del RC2 como input (CCP2)
    
    // Paso 2
    
    PR2 = 155;                  // Establecemos un período de 20mS
    
    // Paso 3
    
    CCP1CONbits.P1M = 0b00;     //Selección del modo Single Output
    
    CCP1CONbits.CCP1M = 0b1100;     // P1A como PWM 
    CCP2CONbits.CCP2M = 0b1111;     // P2A como PWM
            
   // Paso 4
    
    CCP1CONbits.DC1B = 0b11;    
    CCP2CONbits.DC2B1 = 0b1;    
    CCP2CONbits.DC2B0 = 0b1;    // CCPxCON<5:4>
    
    CCPR1L = 11;                // CCPR1L
    CCPR2L = 11;                // CCPR2L
    
                                // Cálculo para 1.5mS de ancho de pulso
    
    // Paso 5
    
    PIR1bits.TMR2IF = 0;        // Bajamos la bandera de interrupción TMR2
    T2CONbits.T2CKPS = 0b11;    // Prescaler de 1:16
    T2CONbits.TMR2ON = 1;       // Se enciende el TMR2
    
    // Paso 6
    
    while(!PIR1bits.TMR2IF){};
    
    TRISCbits.TRISC2 = 0;       // Habilitamos la salida del PWM (RC2)
    TRISCbits.TRISC1 = 0;       // Habilitamos la salida del PWM (RC1)
}