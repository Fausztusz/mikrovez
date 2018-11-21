/*
 * File:   newmainXC16.c
 * Author: student
 *
 * Created on 13 November 2018, 08:38
 */


#include "xc.h"
#include <xc.h>
#include <stdio.h>      //sprintf miatt 
#include <stdlib.h>     
#include <string.h>     //memset


// PIC24HJ128GP502 Configuration Bit Settings
// FOSCSEL
// Oscillator Mode (Internal Fast RC (FRC) w/ PLL)
#pragma config FNOSC = FRCPLL
// Internal External Switch Over Mode (Start-up device with user-selected oscillator source)
#pragma config IESO = OFF
// FOSC
// Primary Oscillator Source (Primary Oscillator Disabled)
#pragma config POSCMD = NONE
// OSC2 Pin Function (OSC2 pin has digital I/O function)
#pragma config OSCIOFNC = ON
// Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are disabled)
#pragma config FCKSM = CSDCMD
// FWDT
// Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)
#pragma config FWDTEN = OFF
// FICD
// JTAG Port Enable (JTAG is Disabled)
#pragma config JTAGEN = OFF

// Hardver tulajdonsagok:
// PLL utani orajel
#define SYS_FREQ		79227500L
#define FCY				SYS_FREQ/2
// Masodlagos orajel forras - orakvarc
#define F_SEC_OSC		32768

// Delay fuggvenyek
#define DELAY_MS(ms)	__delay_ms(ms);
#define DELAY_US(us)	__delay_us(us);
#include <libpic30.h>  // __delay_ms

// 25LC256 SPI EEPROM parancsai
#define SEE_WRSR    1   // st�tusz regiszter �r�sa
#define SEE_WRITE   2   // �r�s parancs
#define SEE_READ    3   // olvas�s parancs
#define SEE_WDI     4   // �r�s tilt�sa parancs
#define SEE_RDSR    5   // st�tusz regiszter olvas�sa
#define SEE_WEN     6   // �r�s enged�lyez�se parancs

//EEPROM CS l�b
#define CSEE _RA2// CS l�b
#define TCSEE _TRISA2 // CS l�b ir�nya

//Makr�k 4 bites bek�t�shez
#define putLCD(c)   writeLCD(c, 1, 1)   //karakter k�ld�se 
#define cmdLCD(d)   writeLCD(d, 0, 1)   //parancs k�ld�se 2 r�szben 
#define cmd1LCD(d)  writeLCD(d, 0, 0)   //parancs k�ld�se 1 r�szben 
#define LCDclr()    writeLCD(0x01,0,1)  //LCD t�rl�se 
#define LCD2row()   writeLCD(0xC0,0,1)  //LCD a 2. sorba 
#define LCDhome()   writeLCD(0x02,0,1)  //LCD az 1. sorba

// LED-ek
#define LEDR		_LATB13
#define LEDG		_LATB12
#define LEDB		_LATB14
#define R_LED		{LEDR=1; LEDG=0; LEDB=0;}
#define G_LED		{LEDR=0; LEDG=1; LEDB=0;}
#define B_LED		{LEDR=0; LEDG=0; LEDB=1;}

#define BAUDRATE 9600
#define BRGVAL ((FCY/BAUDRATE)/16) - 1

//Root password
#define PWD "oc"


/////////<EEPROM>/////////
void InitEE(void){ // EEPROM l�bainak inicializ�l�sa
TCSEE = 0;// CS kimenet
CSEE = 1;// EEPROM elenged�se
}

//1 b�jt k�ld�se �s fogad�sa
int WriteSPI1( int i)
{
SPI1BUF = i;// buffer�r�sa k�ld�sre
while( !SPI1STATbits.SPIRBF);// v�rakoz�s az �tvitel // befejez�s�ig
return SPI1BUF; // be�rkez? adat kiolvas�sa
}

//�r�s enged�lyez�se 
void WriteEnable( void) { 
CSEE = 0; // EEPROM kiv�laszt�sa 
WriteSPI1( SEE_WEN); // �r�s enged�lyez�se parancs 
CSEE = 1; // EEPROM elenged�se 
}

// st�tusz regiszter olvas�sa
int ReadSR( void){
int i;
CSEE = 0;// EEPROM kiv�laszt�sa
WriteSPI1( SEE_RDSR); // st�tusz regiszter olvas�sa
i = WriteSPI1(0);// k�ld�s/fogad�s
CSEE = 1;// EEPROM elenged�se
return i;
};

// 16 bites c�m tartalm�nak olvas�sa 
int ReadEE( int address) { 
    int val; 
    while ( ReadSR() & 0x01);
    // �r�s folyamat�nak v�ge (WIP)
        CSEE = 0; // EEPROM kiv�laszt�sa 
        WriteSPI1( SEE_READ); // olvas�s parancs 
        WriteSPI1( (address>>8) & 0x00ff); // a c�m fels? r�sze (MSB) 
        WriteSPI1( address & 0x00ff); // a c�m als� r�sze (LSB) 
        val = WriteSPI1(0);     // dummy �rt�k k�ld�se/�rt�k beolvas�sa 
        CSEE = 1;               // EEPROM elenged�se 
        return val;
}

//Adat �r�sa a 16 bites c�mre 
void WriteEE( int address, int data) {
    while ( ReadSR() & 0x01); // �r�s folyamat�nak v�ge (WIP)
    WriteEnable(); //�r�s enged�lyez�se
    CSEE = 0; // EEPROM kiv�laszt�sa 
    WriteSPI1( SEE_WRITE); // �r�s parancs 
    WriteSPI1( (address>>8) & 0x00ff); // a c�m fels? r�sze (MSB) 
    WriteSPI1( address & 0x00ff); // a c�m als� r�sze (LSB) 
    WriteSPI1( data & 0x00ff); // az adat k�ld�se 
    CSEE = 1; // EEPROM elenged�se
}
/////////</EEPROM>/////////

///////////<LCD>///////////
//Strobe jel
void pulseE(void){
    DELAY_US(1);
    _LATA1 = 1; //E magas
    DELAY_US(1);
    _LATA1 = 0; //E alacsony
    DELAY_US(1);
}

//LCD l�bak konfigur�l�sa
void configLCD(void){
TRISB = TRISB && 0x0FFF; //D4-D7 kimenet
_TRISA0=0; //RS kimenet
_TRISA1=0; //E kimenet
_LATA0 = 0; //RS alacsony
_LATA1 = 0; //E alacsony
    
}

//Adatvonalak haszn�lata //Adatvonalak haszn�lata //Adatvonalak haszn�lata //Adatvonalak haszn�lata //Adatvonalak haszn�lata //Adatvonalak haszn�lata //Adatvonalak haszn�lata //Adatvonalak haszn�lata //Adatvonalak haszn�lata //Adatvonalak haszn�lata //Adatvonalak haszn�lata //Adatvonalak haszn�lata //Adatvonalak haszn�lata //Adatvonalak haszn�lata //Adatvonalak haszn�lata //Adatvonalak haszn�lata //Adatvonalak haszn�lata //Adatvonalak haszn�lata //Adatvonalak haszn�lata //Adatvonalak haszn�lata //Adatvonalak haszn�lata //Adatvonalak haszn�lata //Adatvonalak haszn�lata //Adatvonalak haszn�lata
void toLCD(char c) {
_LATB12 = c & 0x01; //D4
_LATB13 = (c >> 1)& 0x01; //D5
_LATB14 = (c >> 2)& 0x01; //D6
_LATB15 = (c >> 3)& 0x01; //D7
};

// Adat vagy parancs k�ld�se
// c adat
// rs 0 - parancs, 1 - adat
// bit8 0 - fels? 4bit k�ld�se, 1 - 8 bit k�ld�se 2 r�szletben 
void writeLCD(char c, int rs, int bit8) { 
    
    DELAY_MS(10);  //nem n�zz�k a foglalts�got, csak k�sleltet�nk
    if (rs) _LATA0 = 1;    // adat 
    else _LATA0 = 0;    // parancs 
    toLCD(c >> 4);         // a felso 4 bit k�ld�se
    pulseE(); // strobe 
    if (bit8) { 
        toLCD(c);            // az als� 4 bit k�ld�se 
        pulseE(); // strobe 
    }
} 

//--Ki�r egy karakterf�z�rt az LCD-re 
void putsLCD(char *s) { 
    
while (*s) { 
    char c = *s;
    if (c == '\n')     
        LCD2row();  //Kurzor mozgat�sa a m�sodik sor elej�re 
    else 
        putLCD(c);      //Karakter ki�rat�sa 
    s++; 
    } 
    toLCD(0);    //RGB miatt }
}

void initLCD() {
configLCD(); //az LCD vez�rl? vonalainak be�ll�t�sa
DELAY_MS(20);   //v�r az eszk�z be�ll�s�ra 
cmd1LCD(0x30);  // 4 bites interface 
DELAY_MS(5);    //v�r az eszk�z be�ll�s�ra 
cmd1LCD(0x30);  // 4 bites interface 
DELAY_MS(1);    //v�r az eszk�z be�ll�s�ra 
cmd1LCD(0x30);  // 4 bites interface

cmd1LCD(0x20);  // 4 bites interface 
cmdLCD(0x28);   // 2 soros display, 5x7 karakter 
cmdLCD(0x08);   // dispaly off 
cmdLCD(0x01);   // k�perny? t�rl�se, kurzor alaphelyzetbe �ll�t�s 
cmdLCD(0x06);   // automatikus inkrement�l�s, nem l�pteti a kijelz?t 
cmdLCD(0x0C);   // a display bekapcsol�sa; kurzor �s villog�s kikapcsolva
DELAY_MS(3);
}

void SetupClock(){	
    /*
	* Belso OSC=7.37MHz
	* FOSC=7.37E6/1/2*43/2=79.2275MHz
	* pp 121
	*/
	CLKDIVbits.FRCDIV = 0;	//nincs leosztva az orajel forras

	// PLL modul beallitasa
	CLKDIVbits.PLLPRE = 0;	//N1=2
	PLLFBD = 41;			//M=43
	CLKDIVbits.PLLPOST = 0;	//N2=2

	while (!OSCCONbits.LOCK);	// varakozas PLL kesz-ig

	RCONbits.SWDTEN = 0;		// Watchdog timer ki
}

///////////</LCD>///////////

///////////<UART>///////////


// serial port (UART1, 8, N, 1, CTS/RTS )
void initUART1(void){
    // UART1
    U1MODE = 0; // UART1 alap �llapotban t�rt�n? haszn�lata
    U1STA = 0; // UART1 st�tuszregiszter null�z�sa
    U1BRG = BRGVAL; //Baudrate be�ll�t�sa
    U1MODEbits.UARTEN = 1; // UART enged�lyez�se
    U1STAbits.UTXEN = 1; // K�ld�s enged�lyez�se
}

//karakter k�ld�se
int putUART1(int c){
    while ( U1STAbits.UTXBF); //csak ha TX buffer �res
    U1TXREG = c;
    return c;
}
//karakterf�z�s k�ld�se
void putsUART1(const char *s){
    while( *s) // *s == '\0' -ig
    putUART1( *s++); // karakter k�ld�se
}


//v�rakoz�s �j karakter kiolvas�s�ig
char getUART1( void){
    while ( !U1STAbits.URXDA); // v�rakoz�s �j karakter �rkez�s�re
    return U1RXREG; // karakter kiolvas�sa
}
//Adott sz�m� karakter olvas�sa
char *getsUART1( char *s, int len){
    char *p = s;
    do{
    *s = getUART1(); // v�rakoz�s karakter �rkez�s�re
    putUART1( *s); // echo
        if ( *s=='\n') // \n kihagy�sa
        continue;
        if ( *s=='\r') // kil�p�s a ciklusb�l
        break;
        s++;
        len--;
    } while ( len>1 ); // buffer v�g�ig
        *s = '\0'; // \0 v�gz?d�s? karakterl�nc
    return p;
}

///////////</UART>///////////

///////////<FUNCTIONALITY>///////////
void confTRISB(char a){
    if(a=='s'){
        TRISB=TRISB & 0x0FFF;   //LED kimenet, minden m�s bemenet 
        _TRISB5=1;              //SDI1 bemenet
    }
    else if(a=='u')TRISB=0; //PORTB kimenet
}

int authenticate(char* pwd){
    if(strcmp(PWD,pwd)==0){
        LCDclr();
        putsLCD("PASS"); 
        putsLCD("PASS"); 
        putsLCD("PASS"); 
        putsLCD("PASS"); 
        putsUART1("Access granted\r\n");
        putsUART1("Add new password\r\n");
        return 1;
    }
    else{
        LCDclr();
        putsLCD("DENIED");        
        putsUART1("Access denied\r\n");
        return 0;
    }
}

int saveData(int addr,char newChar){
   confTRISB('s');
   WriteEE(addr, newChar);                 //EEPROM �r�sa 
   confTRISB('e');
   return 0;
}

///////////</FUNCTIONALITY>///////////
int main(void) {
    
SetupClock();
initLCD();                  //LCD modul inicializ�l�sa 

confTRISB('s');
__builtin_write_OSCCONL(OSCCON & 0xbf); //PPSUnLock 
//SPI 
RPINR20bits.SDI1R=5;    //14-es l�b SDI1 
RPOR3bits.RP6R=7;       //15-�s l�b     SDO1 
RPOR3bits.RP7R=8;       //16-os l�b SCLK1 
//SPI inicializ�l�sa 5MHz-es �rajellel 
SPI1CON1 = 0x013A; // master m�d, CKP=0, CKE=1, 8-bites, 1:2, 1:4 
SPI1STAT = 0x8000; // SPI enged�lyez�se, st�tuszok t�rl�se    
InitEE(); //EEPROM inicializ�l�sa

confTRISB('u');
TRISBbits.TRISB11=1; //RX bemenet
RPOR5bits.RP10R=3; //21-es l�b TX
RPINR18bits.U1RXR=11; //22-es l�b RX
__builtin_write_OSCCONL(OSCCON | 0x40); //PPSLock
initUART1(); //UART1 inicializ�l�sa



putsUART1("Program started\r\n"); //Sz�veg kik�ld�se az UART1-re
/**
 * Read Modes
 * 0:General command
 * 1:Waiting for password
 * 2:Set new entry
 * 3:Login
*/
int readMode = 0;
int flag = 0;
int cnt = 0;
int addr = 0;
int lengthArr[10];
int i; 
for( i = 0;i < 10;i++ ) lengthArr[i] = 0;

char c[10];
//Main loop
while(1){
    getsUART1(c,10); //be�rkezo karakterekre v�runk vagy Enterre
    char *s = (char *)&c;
    if( *s == '?') putsUART1("uMOGI Panelr\n");
    if(readMode == 0){
        while (*s) {
            switch(*s) {
                case 'r': R_LED; break; //R LED vil�g�t
                case 'g': G_LED; break; //G LED vil�g�t
                case 'b': B_LED; break; //B LED vil�g�t
                case 'a':               //Add new entry
                    flag = 1; 
                    putsUART1("Login as root\n\r");
                    putsUART1("Password:\r\n");
                    break; 
                default: putsUART1("Invalid command\r\n"); break;
            }
            s++;
        }
    }
    if(readMode == 1) flag = (authenticate(c)) ? 2 : 0;
    if(readMode == 2) {
        while (*s) {
            saveData(addr,*s);
            addr++;
            s++;
        }        
        flag = 0;
        
        DELAY_MS(2000);             //k�sleltet�s 2s
        char* LCD = (char *)malloc(80);
        putsUART1("EEPROM:\r\n");
        sprintf(LCD, "%c",ReadEE(0)); 
        putsUART1(LCD);
        sprintf(LCD, "%c",ReadEE(1));    
        putsUART1(LCD);
        sprintf(LCD, "%c",ReadEE(2));  
        putsUART1(LCD);
        sprintf(LCD, "%c",ReadEE(3));  
        putsUART1(LCD);
        
    }
    readMode = flag;
}


//Byte �r�sa/olvas�sa 
confTRISB('s');
WriteEE(12, 25);                 //EEPROM �r�sa 
WriteEE(11, 10);                 //EEPROM �r�sa 
if ( ReadEE(12) != 25) { R_LED;} //ha nem siker�lt, piros LED 
else G_LED;                       //ha igen, z�ld LED

char* LCD = (char *)malloc(80);
sprintf(LCD, "Szam:\n%i",ReadEE(12));    //Sz�m konvert�l�sa 
LCDclr();                           //LCD t�rl�se 


DELAY_MS(2000);             //k�sleltet�s 2s
sprintf(LCD, "Szam:\n%i",ReadEE(11));    //Sz�m konvert�l�sa 
LCDclr();                           //LCD t�rl�se 
putsLCD(LCD);                       //Sz�veg kik�ld�se az LCD-re

while(1); return (0); }