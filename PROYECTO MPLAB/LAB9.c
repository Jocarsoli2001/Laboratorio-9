/*
 * File:   LAB9.c
 * Author: José Santizo
 *
 * Creado el 4 de octubre de 2021
 * 
 * Descripción: Control de servo motores por modulo PWM
 */

//---------------------Bits de configuración-------------------------------
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


//-----------------Librerías utilizadas en código-------------------- 
#include <xc.h>
#include <stdint.h>
#include <stdio.h>

//-----------------Definición de frecuencia de cristal---------------
#define _XTAL_FREQ 4000000

//-----------------------Constantes----------------------------------
#define  valor_tmr0 237                     // valor_tmr0 = 237

//-----------------------Variables------------------------------------
int cont2 = 0;
int cont_vol = 0;
uint8_t digi = 0;
uint8_t  disp_selector = 0b001;
int dig[3];

//------------Funciones sin retorno de variables----------------------
void setup(void);                           // Función de setup
void divisor(void);                         // Función para dividir números en dígitos
void tmr0(void);                            // Función para reiniciar TMR0
void displays(void);                        // Función para alternar valores mostrados en displays

//-------------Funciones que retornan variables-----------------------
int  tabla(int a);                          // Tabla para traducir valores a displays de 7 segmentos
int  tabla_p(int a);                        // Tabla que traduce valores a displays de 7 segmentos pero con punto decimal incluido

//----------------------Interrupciones--------------------------------
void __interrupt() isr(void){
    if(PIR1bits.ADIF){
        if(ADCON0bits.CHS == 1){            // Si el channel es 1 (puerto AN1)
            cont_vol = 2*ADRESH;            // Valor analógico traducido = cont_vol
            divisor();                      // Llamar subrutina de divisor
        }
        else{
            PORTC = ADRESH;                 // Si channel select = 0
        }                                   //   entonces asignar PORTC = ADRESH
        PIR1bits.ADIF = 0;                  // Limpiar bander de interrupción ADC
    }
    if(T0IF){
        tmr0();                             // Mostrar displays en interrupción de Timer 0
        displays();
    }
}

//----------------------Main Loop--------------------------------
void main(void) {
    setup();                                // Subrutina de setup
    ADCON0bits.GO = 1;                      // Comenzar conversión ADC 
    while(1){
        if(ADCON0bits.GO == 0){             // Si bit GO = 0
            if(ADCON0bits.CHS == 1){        // Si Input Channel = AN1
                ADCON0bits.CHS = 0;         // Asignar input Channel = AN0
                __delay_us(50);             // Delay de 50 ms
            }
            else{                           // Si Input Channel = AN0
                ADCON0bits.CHS = 1;         // Asignar Input Channel = AN1
                __delay_us(50);
            }
            __delay_us(50);
            ADCON0bits.GO = 1;              // Asignar bit GO = 1
        } 
    }
}

//----------------------Subrutinas--------------------------------
void setup(void){
    
    //Configuración de entradas y salidas
    ANSEL = 0b00000011;                     // Pines 0 y 1 de PORTA como analógicos
    ANSELH = 0;
    
    TRISA = 0b00000011;                     // PORTA, bit 0 y 1 como entrada analógica
    TRISC = 0;                              // PORTC como salida
    TRISD = 0;                              // PORTD como salida                           
    TRISE = 0;                              // PORTE como salida
    
    PORTA = 0;                              // Limpiar PORTA
    PORTD = 0;                              // Limpiar PORTD
    PORTC = 0;                              // Limpiar PORTC
    PORTE = 0;                              // Limpiar PORTE
    
    //Configuración de oscilador
    OSCCONbits.IRCF = 0b0110;               // Oscilador a 4 MHz = 110
    OSCCONbits.SCS = 1;
    
    //Configuración de TMR0
    OPTION_REGbits.T0CS = 0;                // bit 5  TMR0 Clock Source Select bit...0 = Internal Clock (CLKO) 1 = Transition on T0CKI pin
    OPTION_REGbits.T0SE = 0;                // bit 4 TMR0 Source Edge Select bit 0 = low/high 1 = high/low
    OPTION_REGbits.PSA = 0;                 // bit 3  Prescaler Assignment bit...0 = Prescaler is assigned to the Timer0
    OPTION_REGbits.PS2 = 1;                 // bits 2-0  PS2:PS0: Prescaler Rate Select bits
    OPTION_REGbits.PS1 = 1;
    OPTION_REGbits.PS0 = 1;
    TMR0 = valor_tmr0;                      // preset for timer register
    
    //Configuración del ADC
    ADCON1bits.ADFM = 0;                    // Resultado justificado a la izquierda
    ADCON1bits.VCFG0 = 0;                   // Voltaje 0 de referencia = VSS
    ADCON1bits.VCFG1 = 0;                   // Voltaje 1 de referencia = VDD
    
    ADCON0bits.ADCS = 0b01;                 // Conversión ADC generada a 2us
    ADCON0bits.CHS = 0;                     // Input Channel = AN0
    ADCON0bits.ADON = 1;                    // ADC = enabled
    __delay_us(50);
    
    //Configuración de interrupciones
    INTCONbits.T0IF = 0;                    // Habilitada la bandera de TIMER 0      
    INTCONbits.T0IE = 1;                    // Habilitar las interrupciones de TIMER 0
    INTCONbits.GIE = 1;                     // Habilitar interrupciones globales
    PIR1bits.ADIF = 0;                      // Limpiar bandera de interrupción del ADC
    PIE1bits.ADIE = 1;                      // Interrupción ADC = enabled
    INTCONbits.PEIE = 1;                    // Interrupciones periféricas activadas
    
    return;
}

void tmr0(void){
    INTCONbits.T0IF = 0;                    // Limpiar bandera de TIMER 0
    TMR0 = valor_tmr0;                      // TMR0 = 237
    return;
}

void divisor(void){
    for(int i = 0; i<3 ; i++){              // De i = 0 hasta i = 2
        dig[i] = cont_vol % 10;             // array[i] = cont_vol mod 10(retornar residuo). Devuelve digito por dígito de un número.
        cont_vol = (cont_vol - dig[i])/10;  // cont_vol = valor sin último digito.
    }
}

void displays(void){
    PORTE = disp_selector;                  // PORTE = 0b001
    if(disp_selector == 0b001){             // Si disp_selector = 0b001
        PORTD = tabla(dig[0]);              // PORTD = valor traducido de posición 0 de array dig[]
        disp_selector = 0b010;              // disp_selector = 0b010
    }
    else if(disp_selector == 0b010){        // Si disp_selector = 0b010
        PORTD = tabla(dig[1]);              // PORTD = valor traducido de posición 1 de array dig[]
        disp_selector = 0b100;              // disp_selector = 0b100
    }
    else if(disp_selector == 0b100){        // Si disp_selector = 0b100
        PORTD = tabla_p(dig[2]);            // PORTD = valor traducido de posición 2 de array dig[]
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