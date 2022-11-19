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