/*
 * File:   LAB9.c
 * Author: Jos� Santizo
 *
 * Creado el 4 de octubre de 2021
 * 
 * Descripci�n: Control de servo motores por modulo PWM
 */

//---------------------Bits de configuraci�n-------------------------------
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
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


//-----------------Librer�as utilizadas en c�digo-------------------- 
#include <xc.h>
#include <stdint.h>
#include <stdio.h>

//-----------------Definici�n de frecuencia de cristal---------------
#define _XTAL_FREQ 8000000

//-----------------------Constantes----------------------------------
#define  valor_tmr0 237                     // valor_tmr0 = 237

//-----------------------Variables------------------------------------
int cont2 = 0;
int cont_vol = 0;
uint8_t digi = 0;
uint8_t  disp_selector = 0b001;
int dig[3];

//------------Funciones sin retorno de variables----------------------
void setup(void);                           // Funci�n de setup
void divisor(void);                         // Funci�n para dividir n�meros en d�gitos
void tmr0(void);                            // Funci�n para reiniciar TMR0
void displays(void);                        // Funci�n para alternar valores mostrados en displays

//-------------Funciones que retornan variables-----------------------
int  tabla(int a);                          // Tabla para traducir valores a displays de 7 segmentos
int  tabla_p(int a);                        // Tabla que traduce valores a displays de 7 segmentos pero con punto decimal incluido

//----------------------Interrupciones--------------------------------
void __interrupt() isr(void){
    if(PIR1bits.ADIF){
        if(ADCON0bits.CHS == 1){            // Si el channel es 1 (puerto AN1)
            CCPR2L = (ADRESH>>1)+118;       // ADRESH = CCPR2L (duty cycle de 118 a 255)
            
        }
        else{                               // Si input channel = 0 (puerto AN0)
            CCPR1L = (ADRESH>>1)+118;       // ADRESH = CCPR1L (duty cycle de 118 a 255)
        }                                   
        PIR1bits.ADIF = 0;                  // Limpiar bander de interrupci�n ADC
    }
    if(T0IF){
        tmr0();                             // Mostrar displays en interrupci�n de Timer 0
        displays();
    }
}

//----------------------Main Loop--------------------------------
void main(void) {
    setup();                                // Subrutina de setup
    ADCON0bits.GO = 1;                      // Comenzar conversi�n ADC 
    while(1){
        if(ADCON0bits.GO == 0){             // Si bit GO = 0
            if(ADCON0bits.CHS == 1){        // Si Input Channel = AN1
                ADCON0bits.CHS = 0;         // Asignar input Channel = AN0
                __delay_us(150);             // Delay de 50 ms
            }
            else{                           // Si Input Channel = AN0
                ADCON0bits.CHS = 1;         // Asignar Input Channel = AN1
                __delay_us(150);
            }
            __delay_us(200);
            ADCON0bits.GO = 1;              // Asignar bit GO = 1
        } 
    }
}

//----------------------Subrutinas--------------------------------
void setup(void){
    
    //Configuraci�n de entradas y salidas
    ANSEL = 0b00000111;                     // Pines 0 y 1 de PORTA como anal�gicos
    ANSELH = 0;
    
    TRISA = 0b00000111;                     // PORTA, bit 0 y 1 como entrada anal�gica
    TRISC = 0;                              // PORTC como salida
    TRISD = 0;                              // PORTD como salida                           
    TRISE = 0;                              // PORTE como salida
    
    PORTA = 0;                              // Limpiar PORTA
    PORTD = 0;                              // Limpiar PORTD
    PORTC = 0;                              // Limpiar PORTC
    PORTE = 0;                              // Limpiar PORTE
    
    //Configuraci�n de oscilador
    OSCCONbits.IRCF = 0b0111;               // Oscilador a 8 MHz = 111
    OSCCONbits.SCS = 1;
    
    //Configuraci�n de TMR0
    OPTION_REGbits.T0CS = 0;                // bit 5  TMR0 Clock Source Select bit...0 = Internal Clock (CLKO) 1 = Transition on T0CKI pin
    OPTION_REGbits.T0SE = 0;                // bit 4 TMR0 Source Edge Select bit 0 = low/high 1 = high/low
    OPTION_REGbits.PSA = 0;                 // bit 3  Prescaler Assignment bit...0 = Prescaler is assigned to the Timer0
    OPTION_REGbits.PS2 = 1;                 // bits 2-0  PS2:PS0: Prescaler Rate Select bits
    OPTION_REGbits.PS1 = 1;
    OPTION_REGbits.PS0 = 1;
    TMR0 = valor_tmr0;                      // preset for timer register
    
    //Configuraci�n del ADC
    ADCON1bits.ADFM = 0;                    // Resultado justificado a la izquierda
    ADCON1bits.VCFG0 = 0;                   // Voltaje 0 de referencia = VSS
    ADCON1bits.VCFG1 = 0;                   // Voltaje 1 de referencia = VDD
    
    ADCON0bits.ADCS = 0b10;                 // Conversi�n ADC generada con FOSC/32
    ADCON0bits.CHS = 0;                     // Input Channel = AN0
    ADCON0bits.ADON = 1;                    // ADC = enabled
    __delay_us(50);
    
    //Configuraci�n de interrupciones
    INTCONbits.T0IF = 0;                    // Habilitada la bandera de TIMER 0      
    INTCONbits.T0IE = 1;                    // Habilitar las interrupciones de TIMER 0
    INTCONbits.GIE = 1;                     // Habilitar interrupciones globales
    PIR1bits.ADIF = 0;                      // Limpiar bandera de interrupci�n del ADC
    PIE1bits.ADIE = 1;                      // Interrupci�n ADC = enabled
    INTCONbits.PEIE = 1;                    // Interrupciones perif�ricas activadas
    
    //Configuraci�n de PWM
    TRISCbits.TRISC2 = 1;                   // RC2/CCP1 como entrada
    TRISCbits.TRISC1 = 1;                   // RC1/CCP2 como entrada
    PR2 = 255;                              // Frecuencia de TMR2 = 2 us
    CCP1CONbits.P1M = 0;                    // Solo una salida en CCP1
    CCP1CONbits.CCP1M = 0b1100;             // Modo de PWM
    CCP2CONbits.CCP2M = 0b1100;             // Modo de PWM para CCP2
    
    CCPR1L = 0x0f;                          // Duty cicle inicial del PWM en CCP1 y CCP2
    CCPR2L = 0x0f;
    CCP2CONbits.DC2B0 = 0;                  // Bits menos significativos de CCP2
    CCP2CONbits.DC2B1 = 0;
    CCP1CONbits.DC1B = 0;                   // Bits menor significativos de CCP1
    
    //Configuraci�n del Timer 2
    PIR1bits.TMR2IF = 0;                    // Limpiar bandera de TMR2
    T2CONbits.T2CKPS = 0b11;                // Prescaler en 1:16
    T2CONbits.TMR2ON = 1;
    
    while(PIR1bits.TMR2IF == 0);            // Esperar un ciclo de TMR2
    PIR1bits.TMR2IF = 0;                    // Limpiar bandera de TMR2
    
    TRISCbits.TRISC2 = 0;                   // Salida 1 del PWM en RC2
    TRISCbits.TRISC1 = 0;                   // Salida 2 del PWM en RC1
    
    return;
}

void tmr0(void){
    INTCONbits.T0IF = 0;                    // Limpiar bandera de TIMER 0
    TMR0 = valor_tmr0;                      // TMR0 = 237
    return;
}

void divisor(void){
    for(int i = 0; i<3 ; i++){              // De i = 0 hasta i = 2
        dig[i] = cont_vol % 10;             // array[i] = cont_vol mod 10(retornar residuo). Devuelve digito por d�gito de un n�mero.
        cont_vol = (cont_vol - dig[i])/10;  // cont_vol = valor sin �ltimo digito.
    }
}

void displays(void){
    PORTE = disp_selector;                  // PORTE = 0b001
    if(disp_selector == 0b001){             // Si disp_selector = 0b001
        PORTD = tabla(dig[0]);              // PORTD = valor traducido de posici�n 0 de array dig[]
        disp_selector = 0b010;              // disp_selector = 0b010
    }
    else if(disp_selector == 0b010){        // Si disp_selector = 0b010
        PORTD = tabla(dig[1]);              // PORTD = valor traducido de posici�n 1 de array dig[]
        disp_selector = 0b100;              // disp_selector = 0b100
    }
    else if(disp_selector == 0b100){        // Si disp_selector = 0b100
        PORTD = tabla_p(dig[2]);            // PORTD = valor traducido de posici�n 2 de array dig[]
        disp_selector = 0b001;              // disp_selector = 0b001
    }
}

int tabla(int a){
    switch (a){                             // Ingresar valor de "a" a switch case
        case 0:                             // Si a = 0
            return 0b00111111;              // devolver valor 0b00111111
            break;
        case 1:                             // Si a = 1
            return 0b00000110;              // devolver valor 0b00000110 
            break;
        case 2:                             // Si a = 2
            return 0b01011011;              // devolver valor 0b01011011
            break;
        case 3:                             // Si a = 3
            return 0b01001111;              // devolver valor 0b01001111
            break;
        case 4:                             // Si a = 4
            return 0b01100110;              // devolver valor 0b01100110
            break;
        case 5:                             // Si a = 5
            return 0b01101101;              // devolver valor 0b01101101
            break;
        case 6:                             // Si a = 6
            return 0b01111101;              // devolver valor 0b01111101
            break;
        case 7:                             // Si a = 7
            return 0b00000111;              // devolver valor 0b01111101
            break;
        case 8:                             // Si a = 8
            return 0b01111111;              // devolver valor 0b01111111
            break;
        case 9:                             // Si a = 9
            return 0b01101111;              // devolver valor 0b01101111
            break;
        default:
            break;
            
    }
}
 
int tabla_p(int a){
    switch (a){                             // Ingresar valor de "a" a switch case
        case 0:                             // Si a = 0
            return 0b10111111;              // devolver valor 0b10111111
            break;
        case 1:                             // Si a = 1
            return 0b10000110;              // devolver valor 0b10000110 
            break;
        case 2:                             // Si a = 2
            return 0b11011011;              // devolver valor 0b11011011
            break;
        case 3:                             // Si a = 3
            return 0b11001111;              // devolver valor 0b11001111
            break;
        case 4:                             // Si a = 4
            return 0b11100110;              // devolver valor 0b11100110
            break;
        case 5:                             // Si a = 5
            return 0b11101101;              // devolver valor 0b11101101
            break;
        case 6:                             // Si a = 6
            return 0b11111101;              // devolver valor 0b11111101
            break;
        case 7:                             // Si a = 7
            return 0b10000111;              // devolver valor 0b11111101
            break;
        case 8:                             // Si a = 8
            return 0b11111111;              // devolver valor 0b11111111
            break;
        case 9:                             // Si a = 9
            return 0b11101111;              // devolver valor 0b11101111
            break;
        default:
            break;
            
    }
}