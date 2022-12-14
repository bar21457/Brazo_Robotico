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

unsigned int ADC1;
unsigned int ADC2;
unsigned int ADC3;
unsigned int ADC4;
unsigned int SERVO1;
unsigned int SERVO2;
unsigned int SERVO3;
unsigned int SERVO4;

uint8_t ADDRESS = 0; 
uint8_t MODO = 0; 
uint8_t DATA;

//******************************************************************************
// Prototipos de Funciones
//******************************************************************************

void setup(void);
void setupADC(void);
void setupPWM(void);

void estado_alto (unsigned int seg);
void conv_s1(int valor);
void conv_s2(int valor);

void write_EEPROM(uint8_t ADDRESS, uint8_t DATA);

uint8_t read_EEPROM(uint8_t ADDRESS);

unsigned int map(uint8_t ADC, int entrada_min, int entrada_max, int salida_min, int salida_max);

//******************************************************************************
// Interrupciones
//******************************************************************************

void __interrupt () isr (void)
{
    if(INTCONbits.T0IF)         // Interrupci??n del TMR0
    {
        INTCONbits.T0IF = 0;    // Limpiamos la bandera del TMR0
        TMR0 = V_TMR0;      // Asignamos el valor del TMR0

        PORTCbits.RC3 = 1;      // Encendemos el RC3
        estado_alto (SERVO3);          

        PORTCbits.RC3 = 0;      // Apagamos el RC3
        PORTCbits.RC4 = 1;      // Encendemos el RC4
        estado_alto (SERVO4);

        PORTCbits.RC4 = 0;      // Apagamos el RC4
        
        /*
         * Se establece que la interrupci??n del TMR0 suceder?? cada 20ms para
         * as?? crear una onda con un per??odo de 20ms cuyos estados alto y bajo
         * indicar??n el grado de rotaci??n al cual se colocar??n los servos en
         * los puertos RC3 y RC4
         */
        
    }
    
    if (INTCONbits.RBIF)        // Interrupci??n del RBIF
    {
        if (PORTBbits.RB0 == 0) // Verificamos si se presion?? RB0
        {
            MODO++;             // Incrementamos en 1 MODO
        }
        
        if (PORTBbits.RB1 == 0) // Verificamos si se presion?? RB1
        {
            ADDRESS = ADDRESS + 4; // Le sumamos 4 a ADDRESS
        }
        
        else if (PORTBbits.RB2 == 0)    // Verificamos si se presion?? RB2
        { 
            ADDRESS = ADDRESS - 4; // Le restamos 4 a ADDRESS
        }

        else if (PORTBbits.RB3 == 0)    // Verificamos si se presion?? RB3
        {
            write_EEPROM(ADDRESS, SERVO1);
            write_EEPROM(ADDRESS + 1, SERVO2);
            write_EEPROM(ADDRESS + 2, SERVO3);
            write_EEPROM(ADDRESS + 3, SERVO4);

            // Reescribimos la EEPROM
        }
        
        else if (MODO == 1)
        {
            if (PORTBbits.RB4 == 0)     // Verificamos si se presion?? RB4
            {
                SERVO1 = read_EEPROM(ADDRESS);
                SERVO2 = read_EEPROM(ADDRESS+1);
                SERVO3 = read_EEPROM(ADDRESS+2);
                SERVO4 = read_EEPROM(ADDRESS+3);

                // Leemos la EEPROM

            }
            else if (PORTBbits.RB1 == 0)    // Verificamos si se presion?? RB1
            {
                ADDRESS = ADDRESS + 4;  // Le sumamos 4 a ADDRESS
            }

            else if (PORTBbits.RB2 == 0)    // Verificamos si se presion?? RB2
            {
                ADDRESS = ADDRESS - 4;  // Le restamos 4 a ADDRESS
            }
        }

        INTCONbits.RBIF = 0;    // Bajamos la bandera de interrupci??n dle RBIF
    }
    return;
}

//******************************************************************************
// C??digo Principal
//******************************************************************************

void main(void) 
{    
    setup();
    setupADC();
    setupPWM();
    
    while(1)
    {    
        //**********************************************************************
        // Modo Manual
        //**********************************************************************

        if (MODO == 0)
        {
            PORTDbits.RD0 = 1;
            PORTDbits.RD1 = 0;
            PORTDbits.RD2 = 0;
            
            //******************************************************************
            // Primer Servomotor
            //******************************************************************
            
            ADCON0bits.CHS = 0b0000;    // Usamos el AN0

            __delay_us(100);

            ADCON0bits.GO = 1;      //Iniciamos la conversi??n en el ADC
            while(ADCON0bits.GO == 1){
                ;
            }

            ADIF = 0;           // Bajamos la bandera de interrupci??n del ADC
            ADC1 = ADRESH;      // Pasamos el valor de ADRESH a ADC
            conv_s1 (ADC1);       
            CCPR1L = SERVO1;     
            
            __delay_us(100);
            
            //******************************************************************
            // Segundo Servomotor
            //******************************************************************

            ADCON0bits.CHS = 0b0001;    // Usamos el AN1
            
            __delay_us(100);
            
            ADCON0bits.GO = 1;      //Iniciamos la conversi??n en el ADC
            while(ADCON0bits.GO == 1){
                ;
            }
           
            ADIF = 0;           // Bajamos la bandera de interrupci??n del ADC
            ADC2 = ADRESH;      // Pasamos el valor de ADRESH a ADC2
            conv_s2 (ADC2);      
            CCPR2L = SERVO2;
            
            __delay_us(100);
            
            //******************************************************************
            // Tercer Servomotor
            //******************************************************************

            ADCON0bits.CHS = 0b0010;    // Usamos el AN2
            
            __delay_us(100);
            
            ADCON0bits.GO = 1;     //Iniciamos la conversi??n en el ADC
            while(ADCON0bits.GO == 1){
                ;
            }
            ADIF = 0;           // Bajamos la bandera de interrupci??n del ADC
            ADC3 = ADRESH;      // Pasamos el valor de ADRESH a ADC3

            SERVO3 = map(ADC3, 0, 255, 5, 17); // Mapeamos el valor de SERVO3
            
            __delay_us(100);
            
            //******************************************************************
            // Cuarto Servomotor
            //******************************************************************
            
            ADCON0bits.CHS = 0b0011;    // Usamos el AN3
            
            __delay_us(100);
            
            ADCON0bits.GO = 1;  //Iniciamos la conversi??n en el ADC
            while(ADCON0bits.GO == 1){
                ;
            }
            ADIF = 0;           // Bajamos la bandera de interrupci??n del ADC
            ADC4 = ADRESH;      // Pasamos el valor de ADRESH a ADC4
            
            SERVO4 = map(ADC4, 0, 255, 5, 17); // Mapeamos el valor de SERVO4
            
            __delay_us(100);  
        }
        
        //**********************************************************************
        // Modo EEPROM
        //**********************************************************************
        
        if (MODO == 1)
        {
            PORTDbits.RD0 = 0;
            PORTDbits.RD1 = 1;
            PORTDbits.RD2 = 0;
            CCPR2L = SERVO2;
            CCPR1L = SERVO1;    
        }
        
        //**********************************************************************
        // Modo UART
        //**********************************************************************
        
        if (MODO == 2)
        {
            PORTDbits.RD0 = 0;
            PORTDbits.RD1 = 0;
            PORTDbits.RD2 = 1;    
        }
        
        if (MODO == 3)
        {
            MODO = 0;   // Regresamos a MODO = 0
        }
    }
    return;
}

//******************************************************************************
// Funciones
//******************************************************************************

void setup (void)
{
    ANSELH = 0;
    
    TRISA = 0;              //Configuraci??n del PORTA como output
    TRISB = 0b00011111;     //Configuraci??n del PORTB como output/input
    TRISC = 0;              //Configuraci??n del PORTC como output
    TRISD = 0;              //Configuraci??n del PORTD como output
    
    PORTA = 0;              //Limpiamos el PORTA
    PORTB = 0;              //Limpiamos el PORTB
    PORTC = 0;              //Limpiamos el PORTC
    PORTD = 0;              //Limpiamos el PORTD
    
    OPTION_REGbits.nRBPU = 0;   // Habilitamos los pull-ups del PORTB
    IOCB = 0b01111111;    
    
    // Configuraci??n del Oscilador Interno a 500KHz
    
    OSCCONbits.IRCF = 0b011 ;   // Selecci??n de los 500KHz
    OSCCONbits.SCS = 1;         // Selecci??n del Oscilador Interno
    
    // Configuraci??n del TMR0 y su Interrupci??n
    
    OPTION_REGbits.T0CS = 0;        // Fosc/4
    OPTION_REGbits.PSA = 0;         // Prescaler para el TMR0
    OPTION_REGbits.PS = 0b011;      // Prescaler 1:16
    
    TMR0 = V_TMR0;                  // Asignamos valor al TMR0
    
    INTCONbits.TMR0IE = 1;  //Habilitamos la interrupci??n del TMR0
    INTCONbits.T0IF = 0;    //Bajamos la bandera de interrupci??n del TMR0
    
    // Interrupciones
    
    INTCONbits.GIE = 1;     //Habilitamos las interrupciones globales (GIE)
    INTCONbits.PEIE = 1;    //Habilitamos las interrupci??n del PEIE
    INTCONbits.RBIF = 1;    //Habilitamos las interrupciones del PORTB (RBIF)
    INTCONbits.RBIE = 0;    //Bajamos la bandera de interrupci??n del PORTB (RBIE) 
}

void setupADC(void)
{    
    //Paso 1: Selecci??n del puerto de entrada
    
    TRISAbits.TRISA0 = 1;       //Configuraci??n del RA0 como input
    ANSELbits.ANS0 = 1;         //Configuraci??n del pin RA0 como an??logo (AN0)
    
    TRISAbits.TRISA1 = 1;       //Configuraci??n del RA1 como input
    ANSELbits.ANS1 = 1;         //Configuraci??n del pin RA1 como an??logo (AN1)
    
    TRISAbits.TRISA2 = 1;       //Configuraci??n del RA2 como input
    ANSELbits.ANS2 = 1;         //Configuraci??n del pin RA2 como an??logo (AN2)
    
    TRISAbits.TRISA3 = 1;       //Configuraci??n del RA3 como input
    ANSELbits.ANS3 = 1;         //Configuraci??n del pin RA3 como an??logo (AN3)
    
    //Paso 2: Configuraci??n del m??dulo ADC
    
    ADCON0bits.ADCS0 = 1;
    ADCON0bits.ADCS1 = 0;       //Fosc/8
    
    ADCON1bits.VCFG0 = 0;       //VDD como voltaje de referencia -
    ADCON1bits.VCFG1 = 0;       //VSS como voltaje de referencia +
    
    ADCON0bits.CHS0 = 0;
    ADCON0bits.CHS1 = 0;
    ADCON0bits.CHS2 = 0;
    ADCON0bits.CHS3 = 0;        //Selecci??n del canal an??logo AN0 (Default)
    
    ADCON1bits.ADFM = 0;        //Justificado hacia la izquierda
    
    ADCON0bits.ADON = 1;        //Habilitamos el ADC
    
    __delay_us(100);            //Delay para adquirir la lectura
}

void setupPWM(void)
{    
    // Paso 1
    
    TRISCbits.TRISC2 = 1;           //Configuraci??n del RC1 como input (CCP1)
    TRISCbits.TRISC1 = 1;           //Configuraci??n del RC2 como input (CCP2)
    
    // Paso 2
    
    PR2 = 155;                  // Establecemos un per??odo de 20mS
    
    // Paso 3
    
    CCP1CONbits.P1M = 0b00;     //Selecci??n del modo Single Output
    
    CCP1CONbits.CCP1M = 0b1100;     // P1A como PWM 
    CCP2CONbits.CCP2M = 0b1111;     // P2A como PWM
            
   // Paso 4
    
    CCP1CONbits.DC1B = 0b11;    
    CCP2CONbits.DC2B1 = 0b1;    
    CCP2CONbits.DC2B0 = 0b1;    // CCPxCON<5:4>
    
    CCPR1L = 11;                // CCPR1L
    CCPR2L = 11;                // CCPR2L
    
                                // C??lculo para 1.5mS de ancho de pulso
    
    // Paso 5
    
    PIR1bits.TMR2IF = 0;        // Bajamos la bandera de interrupci??n TMR2
    T2CONbits.T2CKPS = 0b11;    // Prescaler de 1:16
    T2CONbits.TMR2ON = 1;       // Se enciende el TMR2
    
    // Paso 6
    
    while(!PIR1bits.TMR2IF){};
    
    TRISCbits.TRISC2 = 0;       // Habilitamos la salida del PWM (RC2)
    TRISCbits.TRISC1 = 0;       // Habilitamos la salida del PWM (RC1)
}

void estado_alto (unsigned int seg)
{
    while (seg > 0)
    {
        __delay_us(50);
        seg--;
    }
    
    /* 
     * El valor del ADC mapeado para el servo indica el tiempo que se debe
     * mantener en estado alto que indica en qu?? grado de rotaci??n se va a
     * colocar el servo
     */
    
    return;
}

void conv_s1(int valor)
{
    SERVO1 = (unsigned short) (7+( (float)(13)/(255) ) * (valor-0));
}

void conv_s2(int valor)
{
    SERVO2 = (unsigned short) (7+( (float)(13)/(255) ) * (valor-0));
}

uint8_t read_EEPROM (uint8_t ADDRESS)
{
    while (WR||RD);
    
    EEADR = ADDRESS;
    EECON1bits.EEPGD = 0;
    EECON1bits.RD = 1;
    return EEDAT;    
}

void write_EEPROM(uint8_t ADDRESS, uint8_t DATA)
{
    uint8_t gieStatus;
    while (WR);
    
    EEADR = ADDRESS;
    EEDAT = DATA;
    
    EECON1bits.EEPGD = 0;
    EECON1bits.WREN = 1;

    gieStatus = GIE;
    INTCONbits.GIE = 0;

    EECON2 = 0x55;
    EECON2 = 0xAA;
    
    EECON1bits.WR = 1;
    EECON1bits. WREN = 0;
    
    INTCONbits.GIE = gieStatus;
}

unsigned int map (uint8_t ADC, int entrada_min, int entrada_max, int salida_min, int salida_max){
    return ((ADC - entrada_min)*(salida_max-salida_min)) / ((entrada_max-entrada_min)+salida_min);
}