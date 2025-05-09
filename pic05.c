
// PIC16F18857 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator mode selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT1  // Power-up default value for COSC bits (HFINTOSC (1MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (FSCM timer disabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR pin is Master Clear function)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out reset enable bits (Brown-out Reset Enabled, SBOREN bit is ignored)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) set to 1.9V on LF, and 2.45V on F Devices)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit can be cleared and set only once in software)
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will not cause a reset)

// CONFIG3
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled, SWDTEN is ignored)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4
#pragma config WRT = OFF        // UserNVM self-write protection bits (Write protection off)
#pragma config SCANE = not_available// Scanner Enable bit (Scanner module is not available for use)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR.)

// CONFIG5
#pragma config CP = OFF         // UserNVM Program memory code protection bit (Program Memory code protection disabled)
#pragma config CPD = OFF        // DataNVM code protection bit (Data EEPROM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 1000000 // INTERNAL OSCILLATOR Hz

void pwm6_init(void)
{
    TRISCbits.TRISC5 = 1;       // RC5??????????

    PWM6CONbits.PWM6POL = 0;    // ????????0?
    PWM6DCH = 0;                // ????????
    PWM6DCL = 0;

    PR2 = 0xFF;                 // ????
    T2CONbits.T2CKPS = 0b01;    // ?????? 1:4
    PIR4bits.TMR2IF = 0;        // ??????
    T2CONbits.TMR2ON = 1;       // Timer2??

    while (!PIR4bits.TMR2IF);   // Timer2??????

    RC5PPS = 0x0E;              // PWM6 ? RC5 ????????????????
    TRISCbits.TRISC5 = 0;       // RC5 ??????

    PWM6CONbits.PWM6EN = 1;     // PWM6 ???
}

void pwm6_set_duty(uint16_t duty)
{
    if (duty > 1023) duty = 1023;
    PWM6DCH = duty >> 2;
    PWM6DCL = (duty & 0x03) << 6;
}

void main(void)
{
//    OSCCON1 = 0x60; // HFINTOSC with divider
//    OSCFRQ = 0x05;  // 16 MHz?HFFRQ = 101?
    pwm6_init();

//    TRISCbits.TRISC5 = 0;
    while (1) {
        for (uint16_t i = 0; i < 1024; i++) {
            pwm6_set_duty(i);
            __delay_ms(2);
        }
        for (int16_t i = 1023; i >= 0; i--) {
            pwm6_set_duty(i);
            __delay_ms(2);
        }
//        LATCbits.LATC5 = 1;
//        __delay_ms(500);
//        LATCbits.LATC5 = 0;
//        __delay_ms(500);
    }
}