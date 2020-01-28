// CONFIG1H
#pragma config OSC = INTIO67    // Oscillator Selection bits (Internal oscillator block, port function on RA6 and RA7)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <pic18f4520.h>

#include <stdlib.h>
#include "stdio.h"
#include "string.h"

#define _XTAL_FREQ 8000000  // 1000000

float a;
int di;

char mystring[20];
int lenStr = 0;

char dd[20];


void UART_Initialize() {

    TRISCbits.TRISC6 = 1;            // 
    TRISCbits.TRISC7 = 1;            //  
    /*
           Serial Setting      
     * 1.   Setting Baud rate
     * 2.   choose sync/async mode 
     * 3.   enable Serial port (configures RX/DT and TX/CK pins as serial port pins)
     * 3.5  enable Tx, Rx Interrupt(optional)
     * 4.   Enable Tx & RX
     */
    
    // choose the async mode
    TXSTAbits.SYNC = 0;

    // Setting baud rate
    // using the Baud rate table in data sheet
    BAUDCONbits.BRG16 = 0; // 8-bit           
    TXSTAbits.BRGH = 1;
    SPBRG = 51  ;  // 8000000 / 16 / 9600 - 1 = 51

    // Serial enable
    RCSTAbits.SPEN = 1;              //open serial port
    PIR1bits.TXIF  = 1;
    PIR1bits.RCIF  = 0;
    TXSTAbits.TXEN  = 1;             //Enable Tx
    RCSTAbits.CREN  = 1;             //Enable Rx

    // setting TX/RX interrupt
    PIE1bits.TXIE = 0;              //disable Tx interrupt
    IPR1bits.TXIP = 0;              //Setting Tx as low priority interrupt
    PIE1bits.RCIE = 1;              //Enable Rx interrupt
    IPR1bits.RCIP = 0;              //Setting Rc as low priority interrupt
}

void UART_Write(unsigned char data)  // Output on Terminal
{
    while(!TXSTAbits.TRMT);
    TXREG = data;              //write to TXREG will send data 
}

char *GetString(){ // 
    return mystring;
}

void UART_Write_Text(char* text) { // Output on Terminal, limit:10 chars
    for(int i=0;text[i]!='\0';i++)
        UART_Write(text[i]);
}

void ClearBuffer(){
    for(int i = 0; i < 10 ; i++)
        mystring[i] = '\0';
    lenStr = 0;
}

void MyusartRead()
{
    mystring[lenStr] = RCREG;
    UART_Write(mystring[lenStr]);
    lenStr++;
    lenStr %= 10;
    return ;
}


//void __interrupt(high_priority) ISR()
//void __interrupt () ISR (void)
void __interrupt () ISR (void){
  if(INTCONbits.RBIF == 1)    // RBIF            //Makes sure that it is PORTB On-Change Interrupt
  {
    INTCONbits.RBIE = 0;     // RBIE              //Disable On-Change Interrupt
    if(PORTBbits.RB4 == 1){                  //If ECHO is HIGH
      T1CONbits.TMR1ON = 1;                    //Start Timer
    }
    if(PORTBbits.RB4 == 0)                  //If ECHO is LOW
    {
      T1CONbits.TMR1ON = 0;                    //Stop Timer
      a = (float)((TMR1L | (TMR1H<<8))/58.82);  // 58  //Calculate Distance
      
      //Fosc = 8MHz
      //Time = (TMR1H:TMR1L)*(1/Internal Clock)*Prescaler
      //Internal Clock = Fosc/4 = 8MHz/4 = 2MHz
      //Time = (TMR1H:TMR1L)*2/(2000000) = (TMR1H:TMR1L)/1000000
      //Speed of Sound in Air : 340 m/s = 34000 cm/s
      //d = (34000*Time)/2 = (TMR1H:TMR1L)/(2000000/34000)
      //d = (TMR1H:TMR1L)/58.82 cm
    }
    
    INTCONbits.RBIF = 0;                     //Clear PORTB On-Change Interrupt flag
    INTCONbits.RBIE = 1;                     //Enable PORTB On-Change Interrupt
  }
  
  if(RCIF)
  {
        if(RCSTAbits.OERR)
        {
            CREN = 0;
            Nop();
            CREN = 1;
        }
        
        MyusartRead();
  }
  
  return;
}



void main(void) {
  OSCCONbits.IRCF2 = 1; // setting 8MHz
  OSCCONbits.IRCF1 = 1;
  OSCCONbits.IRCF0 = 1;
  
  // CCP1CON<3:0>=11xx --> PWM mode
  CCP1CONbits.CCP1M3 = 1;  //CCP1CON<3>
  CCP1CONbits.CCP1M2 = 1;  //CCP1CON<2>
  //clear TMR2
  TMR2=0x00;
  //set timer2 on
  T2CONbits.TMR2ON = 1;  //T2CON<2>
  //set rc2 output   // pin 17 : RC2/CCP1
  TRISCbits.TRISC2 = 0;
  // set TMR2 Prescale = 16  // T2CON<1:0>=1x  --> 16
  T2CONbits.T2CKPS1 = 1;  //T2CON<1>  
  //PWM period = (PR2+1)*4*TOSC*TMR2_prescale
  //2ms = (PR2+1)*4*(1/8M)*16
  //2*(10^(-3)) = (PR2+1)*4*(1/8000000)*16
  //(Timer2 Period Register) PR2 = 249
  PR2 = 0xF9;  //249
    
  
  TRISB = 0b00010000;           //RB4 as Input PIN (ECHO)
  //TRISB = 0x00                //RB0 Trigger (33 pin) ; RB4 Echo (37 pin)
  //TRISBbits.TRISB4 = 1;
  TRISD = 0x00; // LCD Pins as Output
  LATD = 0x00;
  ADCON1=0xFF;
  
  INTCONbits.GIE = 1;                      //Global Interrupt Enable
  INTCONbits.RBIF = 0;                     //Clear PORTB On-Change Interrupt Flag
  INTCONbits.RBIE = 1;                     //Enable PORTB On-Change Interrupt

  T1CON = 0x10;                 //Initialize Timer Module

  UART_Initialize();
  
  ClearBuffer();
  UART_Write_Text("  Distance =  ");
  ClearBuffer();
  
  while(1)
  {
    TMR1H = 0;                  //Sets the Initial Value of Timer
    TMR1L = 0;                  //Sets the Initial Value of Timer

    PORTBbits.RB0 = 1;               //TRIGGER HIGH
    __delay_us(10);               //10uS Delay
    PORTBbits.RB0 = 0;               //TRIGGER LOW

    __delay_ms(100); //Waiting for ECHO  //__delay_ms(100);
    a = a + 1; //Error Correction Constant

    if(a>=2 && a<=400)          //Check whether the result is valid or not
    {
      //di = 249 - a/2;  
        if(a<10){
            di = 240;
        }
        else if(a<20){
            di = 160;
        }
        else if(a<30){
            di = 130;
          }
        else if(a<50){
            di = 100;
        }
        else if(a<100){
            di = 50;
        }
        else if(a<200){
            di = 30;
        }
        else if(a<300){
            di = 20;
        }
        else if(a<400){
            di = 10;
        }
      //set duty to CCPR1L , CCP1CON<5> (DC1B1) and CCP1CON<4> (DC1B0)
      CCPR1L = di >> 2;  //duty<9:2>
      CCP1CONbits.DC1B1 = (di & 2) >> 1;  //duty<1>  // & b'00000010'
      CCP1CONbits.DC1B0 = (di & 1);  //duty<0>  // & b'00000001'
      //__delay_ms(50);
        
      
      ClearBuffer();
      sprintf(dd,"%.2f",a);
      UART_Write_Text(dd);
      UART_Write_Text(" cm  ");
      ClearBuffer();

    }
    else
    {   
      ClearBuffer();
      UART_Write_Text("  Out of Range  ");
      ClearBuffer();
    }
    __delay_ms(400);  //測量週期為60ms以上，以防止發射信號對迴響信號的影響
  }
}
