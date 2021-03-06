/*
 * File:   newmainXC16.c
 * Author: Fauszt Andr�s, Kis Levente
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
#define PWD "root"


/////////<EEPROM>/////////
void InitEE(void){ // EEPROM l�bainak inicializ�l�sa
TCSEE = 0;// CS kimenet
CSEE = 1;// EEPROM elenged�se
}

//1 b�jt k�ld�se �s fogad�sa
int WriteSPI1( int i){
	SPI1BUF = i;// buffer�r�sa k�ld�sre
	while( !SPI1STATbits.SPIRBF);// v�rakoz�s az �tvitel // befejez�s�ig
	return SPI1BUF; // be�rkez� adat kiolvas�sa
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

//Adat �r�sa a 16 bites c�mt?l n adattal egy bufferb?l
void WriteEEn( int address, char *data, int n){
    int i=0;
    while ( ReadSR() & 0x01); // �r�s folyamat�nak v�ge (WIP)
    WriteEnable(); // �r�s enged�lyez�se
    CSEE = 0; // EEPROM kiv�laszt�sa
    WriteSPI1( SEE_WRITE); // �r�s parancs
    WriteSPI1( (address>>8) & 0x00ff); // a c�m fels? r�sze (MSB)
    WriteSPI1( address & 0x00ff); // a c�m als� r�sze (LSB)
    for(i=0; i < n; i++)
		WriteSPI1( *data++ ); // az adat k�ld�se
    CSEE = 1; // EEPROM elenged�se
}

// 16 bites c�mt?l n adat olvas�sa data bufferbe
void ReadEEn( int address, char *data, int n){
    int i;
    while ( ReadSR() & 0x01); // �r�s folyamat�nak v�ge (WIP)
    CSEE = 0; // EEPROM kiv�laszt�sa
    WriteSPI1( SEE_READ); // olvas�s parancs
    WriteSPI1( (address>>8) & 0x00ff); // a c�m fels? r�sze (MSB)
    WriteSPI1( address & 0x00ff); // a c�m als� r�sze (LSB)
    for(i=0; i < n; i++)
		*data++ = WriteSPI1(0); // dummy �rt�k k�ld�se/�rt�k beolvas�sa
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

//Adatvonalak haszn�lata 
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

/**
 * Both SPI and UART use the TRISB leg
 * This function switch between the two mode
 * @param mode The identifier char of the mode either 's' or 'e'
 */
void confTRISB(char mode){
    if(mode == 's'){
        TRISB=TRISB & 0x0FFF;   //LED kimenet, minden m�s bemenet 
        _TRISB5 = 1;            //SDI1 bemenet
    }
    else if(mode == 'u')TRISB=0; //PORTB kimenet
}
/**
 * Authenticate the root user with root PWD/Azonos�t�s mesterjelsz�val
 * @param pwd/jelsz�
 * @return 1 if the password is correct 0 if its not
 * @return 1 ha helyes a jelsz� 0 ha helytelen
 */
int authenticate(char* pwd){
    if(strcmp(PWD,pwd) == 0){
        LCDclr();
        putsLCD("PASS"); 
        putsUART1("Access granted\r\n");
        putsUART1("Add new password:\r\n");
        G_LED;
        return 1;
    }
    else{
        LCDclr();
        putsLCD("ACCESS DENIED");        
        putsUART1("Access denied\r\n");
        return 0;
    }
}

/**
 * Save data to a 64bit memory block of the EEPROM/64 bit ment�se az EEPROM-ba
 * @param addr The starting addres of the memory block/Kezd� mem�riac�m
 * @param newEntry The message to save/Mentend� sz�veg
 * @return The address of the end of the block/A blokk v�g�nek c�me
 */
int saveData(int addr,char newEntry[]){
    if(addr < 20*64+1){    
        confTRISB('s');
        WriteEEn(addr,newEntry,64);
        confTRISB('e');

       //Prevents writing message when we initialize the program
       //Amikor let�roljuk a mesterjelsz�t ne jelenjen meg ez az �zenet
       if(strcmp(PWD,newEntry)!=0){
           putsUART1("\rData saved                                           \n\r");
           LCDclr();
           putsLCD("Data saved");
       }
    }
    else putsUART1("\rThe memory is full\n\r");
  return addr + 64;
}
/**
 * Try to authenticate the user with the given password/Felhaszn�l� azonos�t�sa jelsz�val
 * @param addr The current maximum address/A jelenlegi legmagasabb mem�riac�m
 * @param pwd The users password/Jelsz�
 */
void login(int addr,char pwd[]){
    int i;
    confTRISB('s');
    char buff[64];
    //for(i = 0; i <= addr; i += 64){
    for(i = 0; i <= 20*64; i += 64){
        ReadEEn(i,buff,64);
        if(strcmp(buff,pwd) == 0){
            confTRISB('e');
			//The extra spaces clear the line in case of a long previous message
			//A plusz sz�k�z�k t�rlik az el�z� esetlegesen hossz� �zenetet
            putsUART1("\rSuccesful login                                 \n\r");
            LCDclr();
            putsLCD("Succesful login"); 
            G_LED;
            return;
        }
    }
    R_LED;
    putsUART1("\rWrong password                                          \n\r");
    LCDclr();
    putsLCD("Wrong password"); 
}

/**
 * Gets the highest not zero memory block address
 * @return The highest not empty memoryblock's address
 * @return Visszaadja a legnagyobb nem �res mem�riablokk c�m�t
 */
int getAddr(){
    int i;
    char buff[64];
    for(i = 64; i <= 20*64; i += 64){
        ReadEEn(i,buff,64);
        if(strcmp(buff,"") == 0)return i;       
    }
    return 20*64;
}   

///////////</FUNCTIONALITY>///////////
int main(void) {
    
    SetupClock();
    initLCD();                  //LCD modul inicializ�l�sa 

    //Sets up the SPI module for the commuinication with the EEPROM
	//SPI modul bekonfigur�l�sa az EEPROM-mal val� kommunik�ci�hoz
    confTRISB('s');
    __builtin_write_OSCCONL(OSCCON & 0xbf); //PPSUnLock 
    //SPI 
    RPINR20bits.SDI1R=5;    //14-es l�b SDI1 
    RPOR3bits.RP6R=7;       //15-�s l�b SDO1 
    RPOR3bits.RP7R=8;       //16-os l�b SCLK1 
    //SPI inicializ�l�sa 5MHz-es �rajellel 
    SPI1CON1 = 0x013A; // master m�d, CKP=0, CKE=1, 8-bites, 1:2, 1:4 
    SPI1STAT = 0x8000; // SPI enged�lyez�se, st�tuszok t�rl�se    
    InitEE(); //EEPROM inicializ�l�sa

    //Sets up the UART commuinication
	//UART kommunik�ci� be�ll�t�sa
    confTRISB('u');
    TRISBbits.TRISB11=1; //RX bemenet
    RPOR5bits.RP10R=3; //21-es l�b TX
    RPINR18bits.U1RXR=11; //22-es l�b RX
    __builtin_write_OSCCONL(OSCCON | 0x40); //PPSLock
    initUART1(); //UART1 inicializ�l�sa



    putsUART1("Program started\r\n"); //Sz�veg kik�ld�se az UART1-re
    /**
     * @readMode
     * 0:Waiting for general command/V�rakoz�s �ltal�nos utas�t�sra
     * 1:Waiting for master password/V�rakoz�s a mesterjelsz�ra
     * 2:Waiting to set new entry/V�rakoz�s az �j jelsz�ra
     * 3:Waiting to normal user password/V�rakoz�s a norm�l felhaszn�l�i jelsz�ra
    */
    int readMode = 0;
    /**
     *  @addr
     *  The currently used highest memory address
     *  A jelenleg haszn�lt legmagasabb mem�riac�m
     */
	int addr = 0;
    int flag = 0;
    int i; 

    //Puts master password to memory
    confTRISB('s');
    addr = saveData(addr,PWD);
    addr = getAddr();
    confTRISB('e');
    B_LED;
    char c[64];
    //Main loop
    while(1){
        getsUART1(c,64); //be�rkezo karakterekre v�runk vagy Enterre
        if(readMode == 0){
            switch(c[0]) {
                   case 'a':               //Add new password entry
                        flag = 1; 
                        putsUART1("Login as root\n\r");
                        putsUART1("Password:\r\n");
                        LCDclr();
                        putsLCD("Login as root");
                        break; 
                    case 'l': 
                        flag = 3;
                        LCDclr();
                        putsLCD("Enter password");
                        putsUART1("Enter password:\n\r");
                        break;
                    case '?':
                        putsUART1(
                            "uMOGI panel\n\r\
                            \rCommand list:\n\r\
                            \r'a\':Login as admin and add new entry\n\r\
                            \r'l\':Login as normal user\n\r\
                            \r'?\':Display this message\n\r"
                            );
                        break;
                    default:
                        putsUART1("Invalid command\r\n");
                        LCDclr();
                        putsLCD("Invalid command");   
                        break;
                }
                
            }
        if(readMode == 1) flag = (authenticate(c)) ? 2 : 0;
        if(readMode == 2) {
            if(strcmp(c,"clear") == 0){
                putsUART1("Reseting memory this could take a few seconds\r");
                LCDclr();
                putsLCD("Reseting memory"); 
                confTRISB('s');
                for(i = 64; i <= 20*64; i++)WriteEE(i,0);//Resets memory
                confTRISB('e');
                putsUART1("Memory reseted                                \n\r");
                addr = 64;
                LCDclr();
                putsLCD("Memory reseted"); 
            } 
            else addr = saveData(addr,c);   //Save data to the EEPROM
            flag = 0; 
        }
        if(readMode == 3) {login(addr, c);flag = 0; }
        
        if(readMode == 0)B_LED;
        readMode = flag;
    }
    return (0); 
}