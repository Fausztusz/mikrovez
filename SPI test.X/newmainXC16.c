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
#define SEE_WRSR    1   // státusz regiszter írása
#define SEE_WRITE   2   // írás parancs
#define SEE_READ    3   // olvasás parancs
#define SEE_WDI     4   // írás tiltása parancs
#define SEE_RDSR    5   // státusz regiszter olvasása
#define SEE_WEN     6   // írás engedélyezése parancs

//EEPROM CS láb
#define CSEE _RA2// CS láb
#define TCSEE _TRISA2 // CS láb iránya

//Makrók 4 bites bekötéshez
#define putLCD(c)   writeLCD(c, 1, 1)   //karakter küldése 
#define cmdLCD(d)   writeLCD(d, 0, 1)   //parancs küldése 2 részben 
#define cmd1LCD(d)  writeLCD(d, 0, 0)   //parancs küldése 1 részben 
#define LCDclr()    writeLCD(0x01,0,1)  //LCD törlése 
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
void InitEE(void){ // EEPROM lábainak inicializálása
TCSEE = 0;// CS kimenet
CSEE = 1;// EEPROM elengedése
}

//1 bájt küldése és fogadása
int WriteSPI1( int i)
{
SPI1BUF = i;// bufferírása küldésre
while( !SPI1STATbits.SPIRBF);// várakozás az átvitel // befejezéséig
return SPI1BUF; // beérkez? adat kiolvasása
}

//Írás engedélyezése 
void WriteEnable( void) { 
CSEE = 0; // EEPROM kiválasztása 
WriteSPI1( SEE_WEN); // írás engedélyezése parancs 
CSEE = 1; // EEPROM elengedése 
}

// státusz regiszter olvasása
int ReadSR( void){
int i;
CSEE = 0;// EEPROM kiválasztása
WriteSPI1( SEE_RDSR); // státusz regiszter olvasása
i = WriteSPI1(0);// küldés/fogadás
CSEE = 1;// EEPROM elengedése
return i;
};

// 16 bites cím tartalmának olvasása 
int ReadEE( int address) { 
    int val; 
    while ( ReadSR() & 0x01);
    // írás folyamatának vége (WIP)
        CSEE = 0; // EEPROM kiválasztása 
        WriteSPI1( SEE_READ); // olvasás parancs 
        WriteSPI1( (address>>8) & 0x00ff); // a cím fels? része (MSB) 
        WriteSPI1( address & 0x00ff); // a cím alsó része (LSB) 
        val = WriteSPI1(0);     // dummy érték küldése/érték beolvasása 
        CSEE = 1;               // EEPROM elengedése 
        return val;
}

//Adat írása a 16 bites címre 
void WriteEE( int address, int data) {
    while ( ReadSR() & 0x01); // írás folyamatának vége (WIP)
    WriteEnable(); //Írás engedélyezése
    CSEE = 0; // EEPROM kiválasztása 
    WriteSPI1( SEE_WRITE); // írás parancs 
    WriteSPI1( (address>>8) & 0x00ff); // a cím fels? része (MSB) 
    WriteSPI1( address & 0x00ff); // a cím alsó része (LSB) 
    WriteSPI1( data & 0x00ff); // az adat küldése 
    CSEE = 1; // EEPROM elengedése
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

//LCD lábak konfigurálása
void configLCD(void){
TRISB = TRISB && 0x0FFF; //D4-D7 kimenet
_TRISA0=0; //RS kimenet
_TRISA1=0; //E kimenet
_LATA0 = 0; //RS alacsony
_LATA1 = 0; //E alacsony
    
}

//Adatvonalak használata //Adatvonalak használata //Adatvonalak használata //Adatvonalak használata //Adatvonalak használata //Adatvonalak használata //Adatvonalak használata //Adatvonalak használata //Adatvonalak használata //Adatvonalak használata //Adatvonalak használata //Adatvonalak használata //Adatvonalak használata //Adatvonalak használata //Adatvonalak használata //Adatvonalak használata //Adatvonalak használata //Adatvonalak használata //Adatvonalak használata //Adatvonalak használata //Adatvonalak használata //Adatvonalak használata //Adatvonalak használata //Adatvonalak használata
void toLCD(char c) {
_LATB12 = c & 0x01; //D4
_LATB13 = (c >> 1)& 0x01; //D5
_LATB14 = (c >> 2)& 0x01; //D6
_LATB15 = (c >> 3)& 0x01; //D7
};

// Adat vagy parancs küldése
// c adat
// rs 0 - parancs, 1 - adat
// bit8 0 - fels? 4bit küldése, 1 - 8 bit küldése 2 részletben 
void writeLCD(char c, int rs, int bit8) { 
    
    DELAY_MS(10);  //nem nézzük a foglaltságot, csak késleltetünk
    if (rs) _LATA0 = 1;    // adat 
    else _LATA0 = 0;    // parancs 
    toLCD(c >> 4);         // a felso 4 bit küldése
    pulseE(); // strobe 
    if (bit8) { 
        toLCD(c);            // az alsó 4 bit küldése 
        pulseE(); // strobe 
    }
} 

//--Kiír egy karakterfüzért az LCD-re 
void putsLCD(char *s) { 
    
while (*s) { 
    char c = *s;
    if (c == '\n')     
        LCD2row();  //Kurzor mozgatása a második sor elejére 
    else 
        putLCD(c);      //Karakter kiíratása 
    s++; 
    } 
    toLCD(0);    //RGB miatt }
}

void initLCD() {
configLCD(); //az LCD vezérl? vonalainak beállítása
DELAY_MS(20);   //vár az eszköz beállására 
cmd1LCD(0x30);  // 4 bites interface 
DELAY_MS(5);    //vár az eszköz beállására 
cmd1LCD(0x30);  // 4 bites interface 
DELAY_MS(1);    //vár az eszköz beállására 
cmd1LCD(0x30);  // 4 bites interface

cmd1LCD(0x20);  // 4 bites interface 
cmdLCD(0x28);   // 2 soros display, 5x7 karakter 
cmdLCD(0x08);   // dispaly off 
cmdLCD(0x01);   // képerny? törlése, kurzor alaphelyzetbe állítás 
cmdLCD(0x06);   // automatikus inkrementálás, nem lépteti a kijelz?t 
cmdLCD(0x0C);   // a display bekapcsolása; kurzor és villogás kikapcsolva
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
    U1MODE = 0; // UART1 alap állapotban történ? használata
    U1STA = 0; // UART1 státuszregiszter nullázása
    U1BRG = BRGVAL; //Baudrate beállítása
    U1MODEbits.UARTEN = 1; // UART engedélyezése
    U1STAbits.UTXEN = 1; // Küldés engedélyezése
}

//karakter küldése
int putUART1(int c){
    while ( U1STAbits.UTXBF); //csak ha TX buffer üres
    U1TXREG = c;
    return c;
}
//karakterfüzés küldése
void putsUART1(const char *s){
    while( *s) // *s == '\0' -ig
    putUART1( *s++); // karakter küldése
}


//várakozás új karakter kiolvasásáig
char getUART1( void){
    while ( !U1STAbits.URXDA); // várakozás új karakter érkezésére
    return U1RXREG; // karakter kiolvasása
}
//Adott számú karakter olvasása
char *getsUART1( char *s, int len){
    char *p = s;
    do{
    *s = getUART1(); // várakozás karakter érkezésére
    putUART1( *s); // echo
        if ( *s=='\n') // \n kihagyása
        continue;
        if ( *s=='\r') // kilépés a ciklusból
        break;
        s++;
        len--;
    } while ( len>1 ); // buffer végéig
        *s = '\0'; // \0 végz?dés? karakterlánc
    return p;
}

///////////</UART>///////////

///////////<FUNCTIONALITY>///////////
void confTRISB(char a){
    if(a=='s'){
        TRISB=TRISB & 0x0FFF;   //LED kimenet, minden más bemenet 
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
   WriteEE(addr, newChar);                 //EEPROM írása 
   confTRISB('e');
   return 0;
}

///////////</FUNCTIONALITY>///////////
int main(void) {
    
SetupClock();
initLCD();                  //LCD modul inicializálása 

confTRISB('s');
__builtin_write_OSCCONL(OSCCON & 0xbf); //PPSUnLock 
//SPI 
RPINR20bits.SDI1R=5;    //14-es láb SDI1 
RPOR3bits.RP6R=7;       //15-ös láb     SDO1 
RPOR3bits.RP7R=8;       //16-os láb SCLK1 
//SPI inicializálása 5MHz-es órajellel 
SPI1CON1 = 0x013A; // master mód, CKP=0, CKE=1, 8-bites, 1:2, 1:4 
SPI1STAT = 0x8000; // SPI engedélyezése, státuszok törlése    
InitEE(); //EEPROM inicializálása

confTRISB('u');
TRISBbits.TRISB11=1; //RX bemenet
RPOR5bits.RP10R=3; //21-es láb TX
RPINR18bits.U1RXR=11; //22-es láb RX
__builtin_write_OSCCONL(OSCCON | 0x40); //PPSLock
initUART1(); //UART1 inicializálása



putsUART1("Program started\r\n"); //Szöveg kiküldése az UART1-re
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
    getsUART1(c,10); //beérkezo karakterekre várunk vagy Enterre
    char *s = (char *)&c;
    if( *s == '?') putsUART1("uMOGI Panelr\n");
    if(readMode == 0){
        while (*s) {
            switch(*s) {
                case 'r': R_LED; break; //R LED világít
                case 'g': G_LED; break; //G LED világít
                case 'b': B_LED; break; //B LED világít
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
        
        DELAY_MS(2000);             //késleltetés 2s
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


//Byte írása/olvasása 
confTRISB('s');
WriteEE(12, 25);                 //EEPROM írása 
WriteEE(11, 10);                 //EEPROM írása 
if ( ReadEE(12) != 25) { R_LED;} //ha nem sikerült, piros LED 
else G_LED;                       //ha igen, zöld LED

char* LCD = (char *)malloc(80);
sprintf(LCD, "Szam:\n%i",ReadEE(12));    //Szám konvertálása 
LCDclr();                           //LCD törlése 


DELAY_MS(2000);             //késleltetés 2s
sprintf(LCD, "Szam:\n%i",ReadEE(11));    //Szám konvertálása 
LCDclr();                           //LCD törlése 
putsLCD(LCD);                       //Szöveg kiküldése az LCD-re

while(1); return (0); }