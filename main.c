/* 
 * File:   main.c
 * Author: joseg
 *
 * Created on May 18, 2022, 3:36 PM
 */

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

#include <xc.h>
#include <stdint.h>

/*------------------------------------------------------------------------------
 * CONSTANTES 
 ------------------------------------------------------------------------------*/
#define _XTAL_FREQ 1000000

/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
uint8_t potenciometro = 0;
uint8_t sleep_mode = 0, address = 0;

/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
void setup(void);
uint8_t read_EEPROM(uint8_t address);
void write_EEPROM(uint8_t address, uint8_t data);

/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    if(PIR1bits.ADIF){              // Fue interrupci?n del ADC?
        potenciometro = ADRESH;              //El potenciometro toma el valor leido
        PORTC = potenciometro;               //Mostrarlo en el puerto D
        PIR1bits.ADIF = 0;          // Limpiamos bandera de interrupci?n
    }
    else if(INTCONbits.RBIF){            // Fue interrupci?n del PORTB  
        if(!PORTBbits.RB0){                     // Verificamos si fue RB0 quien gener? la interrupci?
                sleep_mode = 1;     // Dormimos al pic
            }
        else if(!PORTBbits.RB1){                     // Verificamos si fue RB0 quien gener? la interrupci?n
            sleep_mode = 0;                         //Si estamos en sleep, despertar
        }
        
        else if (!PORTBbits.RB2){
            if(sleep_mode == 1){
            sleep_mode = 0;}
            write_EEPROM(address, potenciometro);
        }
        INTCONbits.RBIF = 0;        // Limpiamos bandera de interrupci?n del puerto B
    }
    return;
}   

/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){ 
        if(ADCON0bits.GO == 0){  //Conversi?n ADC
            ADCON0bits.GO = 1;       
        }
        if(sleep_mode == 1){       //si la bandera esta activada deshabilitar la interrupci?n ADC        
            PIE1bits.ADIE = 0;
            SLEEP();                  // Pasar al modo sleep
        }
        else if(sleep_mode == 0){            
            PIE1bits.ADIE = 1;
        }
        PORTD = read_EEPROM(address);
    }
    return;
}

/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){
    ANSEL = 0b00000010;
    ANSELH = 0;
    
    TRISB = 0xff;
    PORTB = 0;
    
    TRISA = 0b00000010;
    TRISC = 0;
    TRISD = 0;
    
    PORTC = 0;
    PORTD = 0;
    PORTA = 0;
    
    // Configuracion ADC
    ADCON0bits.ADCS = 0b00;     // Fosc/2
    ADCON1bits.VCFG0 = 0;       // VDD
    ADCON1bits.VCFG1 = 0;       // VSS
    ADCON0bits.CHS = 1;    // Seleccionamos el AN1
    ADCON1bits.ADFM = 0;        // Justificado a la izquierda
    ADCON0bits.ADON = 1;        // Habilitamos modulo ADC
    __delay_us(40);             // Sample time

        // Configuracion interrupciones
    PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
    PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.GIE = 1;         // Habilitamos int. globales
    OPTION_REGbits.nRBPU = 0; 
    WPUB = 0xff;
    INTCONbits.RBIE = 1;   
    IOCB = 0xff;         
    INTCONbits.RBIF = 0;
}

//Funci?n para leer
uint8_t read_EEPROM(uint8_t address){
    EEADR = address;
    EECON1bits.EEPGD = 0;  // Lectura en la EEPROM
    EECON1bits.RD = 1;       // Conseguimos dato de la EEPROM
    return EEDAT;              // Regresamos ese dato leido 
}

//Funci?n para escribir
void write_EEPROM(uint8_t address, uint8_t data){
    EEADR = address;
    EEDAT = data;
    EECON1bits.EEPGD = 0; // Escritura en la EEPROM
    EECON1bits.WREN = 1;  // Habilitamos la escritura a la EEPROM
    
    INTCONbits.GIE = 0;    // Deshabilitamos las interrupciones
    EECON2 = 0x55;      
    EECON2 = 0xAA;
    
    EECON1bits.WR = 1;    // Se inicia la escritura
    
    EECON1bits.WREN = 0;     // se deshabilita escritura en la EEPROM
    INTCONbits.RBIF = 0;
    INTCONbits.GIE = 1;   // Habilitamos las interrupciones
}
