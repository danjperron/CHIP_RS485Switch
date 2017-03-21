/*
 * File:   chip_rs485switch.c
 * Author: Daniel
 *
 * Created on March 20, 2017, 2:57 PM
 */
///////////////////////////////////////////////////////////////////////////////////
//  Simple switcher of RS-485 I.C. using i2c to bypass boot log from chip on serial
//
//
//   Date: 21 March 2017
//   programmer: Daniel Perron
//   Version: 1.0
//   Processor: PIC12F1840
//   Software: Microchip MPLAB IDE v8.90  XC8  (freeware version)
//   

/*

The MIT License (MIT)

Copyright (c)  2013 Daniel Perron

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/







/*  CHIP_RS485 Command and pin description

   RA0  TXM SIGNAL    TXM DATA from CHIP
   RA1  SCL
   RA2  SDA
   RA3  MCLR
   RA4  RE            Receive ENABLE to RS845 IC
   RA5  DE            Transmit ENABLE tp RS495 IC
 */
  
#define IN_ENABLE RA5
#define OUT_ENABLE RA4


  
/*
Commands,

00 : Control (R/W)
        (8 bits)
         bit 0:   	Enable/Disable   0:disable  1: Enable    On disable the RS-485 RE and DE are disabled.
         bit 1..7:      not used.

01 : BaudRate (R/W)
     (8 bits)
       bit(0..2):     0 = 1200
                      1 = 2400
                      2 = 4800
                      3 = 9600
                      4 = 19200
                      5 = 38400
                      6 = 57600
                      7 = 115200
       bit(3..7):     not used
02:
03:
04:   N/A

05:  eeprom address  (W) write to flash i2c Address (Default = 0x20)
    (8bits)
     bit 7..0:     I2C Address. (7 bits mode)   117 devices possible.
                      Addresss has to be between (0x3.. 0x77) inclusively
                   N.B. For protection against communication error, The I2C Address will change only if it is Flash (Command  9).
06:  N/A
07:  N/A
08:  oscillator  tune register (R/W)
     This is to adjust the internal PIC OScillator
    (8 bits signed )   (value between -32 to 31)
     N.B. this is a signed 6 bits register ! Do not use value outside (-32 to 31)

09: Flash Settings . (8bits). (Write only)
        (16bits)
        value as to be 0xAA55 otherwise the writing is not done

        This will store the Baud rate into the eeprom.

14 :  Model Version (R)
     (16bits(
              bit 0..7:     return 0x85
              bit 8..15:   return 0x04

15:   Software Version(R)
              bit 0..7: minor release version
              bit 8..15: major release version





*/



#ifdef __XC__
#include <xc.h>

#else
#include <htc.h>
#endif



#ifndef _XTAL_FREQ
 // Unless specified elsewhere, 4MHz system frequency is assumed
 #define _XTAL_FREQ 32000000
#endif


#ifdef __XC__
// CONFIG1
#pragma config FOSC = INTOSC // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF // Internal/External Switchover (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = ON // PLL Enable (4x PLL enabled)
#pragma config STVREN = ON // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = ON // Low-Voltage Programming Enable (Low-voltage programming enabled)

#else


#ifndef BORV_LO
#define BORV_LO BORV_19
#endif


__CONFIG(FOSC_INTOSC & WDTE_OFF & PWRTE_OFF & MCLRE_ON & BOREN_OFF & CP_OFF & CPD_OFF & CLKOUTEN_OFF & IESO_OFF & FCMEN_OFF);
__CONFIG(WRT_OFF & PLLEN_ON & BORV_LO & LVP_ON);
__IDLOC(0000);

#endif


#define IDTAGH  0x04
#define IDTAGL  0x85
#define MAJOR_VERSION 1
#define MINOR_VERSION 0



// time out table
//
// version 1.01  1.3 character  delay instead of 2 full character
//
//  8 bit , 1 stop + 1 start = 10 bit
//  1200 BAUD =  8333 us  * 1.3  = 10833 us    1 BAUD = 833 us   PRE=  7   TIMEout= 255  ****  1200 baud is not possible
//  2400 BAUD =  4167 us  * 1.3  =  5417 us    1 BAUD = 417 us	 PRE=  7   TIMEout= 169
//  4800 BAUD =  2083 us  * 1.3  =  2708 us    1 BAUD = 208 us	 PRE=  6   TIMEout= 169
//  9600 BAUD =  1042 us  * 1.3  =  1354 us    1 BAUD = 104 us	 PRE=  6   TIMEout= 85
// 19200 BAUD =   521 us  * 1.3  =    677 us    1 BAUD =   50 us PRE=  5   TIMEout=  85
// 38400 BAUD =   260 us  * 1.3  =    339 us    1 BAUD =   26 us PRE=  4    TIMEout=  85
// 57600 BAUD =   174 us *  1.3  =    226 us    1 BAUD =   17 us PRE=  3    TIMEout=  113
//115200 BAUD =   87  us *  1.3  =    113 us    1 BAUD =   8.7 us PRE= 3     TIMEout= 57
// RCV_DELAY is now  1 BAUD duration so a  total of 1.4 character length delay after last low level transmission
// PRESCALER 3  (clock/4/16)=   500Khz => 2us
// PRESCALER 4  (clock/4/32)=   250Khz => 4us
// PRESCALER 5  (clock/4/64) =  125KHz => 8us
// PRESCALER 6  (clock/4/128) = 62.5KHz => 16us
// PRESCALER 7  (clock/4/256) = 31.25KHz => 32us


//
//
//
//
//
//
//
//
//
//
//
//
#define  T0_1200_PRESCALER 7
#define  T0_1200_TIME_OUT  0
#define  T0_1200_RCV_DELAY (256-169)

#define  T0_2400_PRESCALER 7
#define  T0_2400_TIME_OUT  (256-169)
#define  T0_2400_RCV_DELAY (256-85)

#define  T0_4800_PRESCALER 6
#define  T0_4800_TIME_OUT (256 - 169)
#define  T0_4800_RCV_DELAY (256 - 85)

#define T0_9600_PRESCALER 6
#define T0_9600_TIME_OUT  (256-85)
#define T0_9600_RCV_DELAY (256-43)

#define T0_19200_PRESCALER 5
#define T0_19200_TIME_OUT  (256-85)
#define T0_19200_RCV_DELAY (256-43)

#define  T0_38400_PRESCALER 4
#define  T0_38400_TIME_OUT (256-85)
#define  T0_38400_RCV_DELAY (256-43)

#define T0_57600_PRESCALER 3
#define T0_57600_TIME_OUT  (256-113)
#define T0_57600_RCV_DELAY (256-56)

#define T0_115200_PRESCALER 3
#define T0_115200_TIME_OUT  (256-56)
#define T0_115200_RCV_DELAY (256-28)


                                     // 1200,2400,4800,9600,19200,38400,57600,115200
const unsigned char BaudPrescaler[8]= { T0_1200_PRESCALER,\
					T0_2400_PRESCALER,\
					T0_4800_PRESCALER,\
					T0_9600_PRESCALER,\
                                        T0_19200_PRESCALER,\
					T0_38400_PRESCALER,\
                                        T0_57600_PRESCALER,\
                                        T0_115200_PRESCALER};

const unsigned char BaudTimeOut[8]= {   T0_1200_TIME_OUT,\
                                        T0_2400_TIME_OUT,\
                                        T0_4800_TIME_OUT,\
                                        T0_9600_TIME_OUT,\
                                        T0_19200_TIME_OUT,\
                                        T0_38400_TIME_OUT,\
                                        T0_57600_TIME_OUT,\
                                        T0_115200_TIME_OUT};


const unsigned char BaudRcvDelay[8]= {  T0_1200_RCV_DELAY,\
                                        T0_2400_RCV_DELAY,\
                                        T0_4800_RCV_DELAY,\
                                        T0_9600_RCV_DELAY,\
                                        T0_19200_RCV_DELAY,\
					T0_38400_RCV_DELAY,\
                                        T0_57600_RCV_DELAY,\
                                        T0_115200_RCV_DELAY};

near volatile unsigned char T0_TimeOut;
near volatile unsigned char T0_RcvDelay;
near volatile unsigned char T0_Prescaler;



////////////////   eerom storage 

__EEPROM_DATA(0x41,7,0xff,0xff,0xff,0xff,0xff,0xff);  // needs to have 8 byte. This only hold the I2C Address and OSCTUNE for now

// set the __EEPROM_DATA  according to the following structure
typedef struct {
unsigned char  I2C_Address;
char OscTune;
unsigned char  BaudRate;
}EepromSettingsStruct;

EepromSettingsStruct Settings;

// I know this is overkill for 2 bytes but this is my generic function
// just in case we add more stuff into Settings
volatile bit SaveSettingsFlag; 

void LoadSettings(void)
{
  unsigned char   idx;
  unsigned char   * pointer = (unsigned char *) &Settings;

  for(idx=0; idx < sizeof(Settings);idx++)
     *(pointer++) = eeprom_read(idx);
  SaveSettingsFlag=0;
}

void SaveSettings(void)
{
  unsigned char   idx;
  unsigned char   * pointer = (unsigned char *) &Settings;

  for(idx=0; idx < sizeof(Settings);idx++)
      eeprom_write(idx, *(pointer++));
  SaveSettingsFlag= 0;
}

                                 // interruption ask to save settings
near unsigned char Command;				// This is the command you want to run from the I2C


near volatile bit CommandEnable;                		     // This is the ENABLE/DISABLE bit
unsigned char BaudRate;

typedef union{
  unsigned char  byte[4];
  unsigned short word[2];
  unsigned long  dword;
}IntegersStruct;



////////  I2C handling   variable


near volatile bit   GotCommandFlag=0;		// I2C interrupts flag to specify we have the command bytes. We wont use bit since we don't want to use the same byte to hold the bit
near volatile unsigned char I2CCommand;		// I2C Command
near volatile unsigned char I2CByteCount;	// I2C Byte Data counter. This is use to verify when we should increment First out counter
near volatile  unsigned short I2CShortData;	// temporary variable to hold i2c short value;

//////////  I2C Initialization routine
void I2CInit()
{
SSP1IE=0;
SSP1CON1=0;
// i2c init
SSP1ADD  = Settings.I2C_Address << 1;
SSP1CON1 = 0b00110110;
SSP1CON2 = 0b00000001;
SSP1CON3 = 0b00000011;
SSP1STAT = 0b11000000;
SSP1MSK  = 0xff;
SSP1IF=0;
// irq
SSP1IE =1;
GIE =1;
PEIE =1;
}


/////////////  I2C Write byte routine
void I2CWrite(unsigned char value)
{
   while(SSP1STATbits.BF);
    do
    {
    SSP1CON1bits.WCOL=0;
    SSP1BUF = value;
    if(SSP1CON1bits.WCOL) continue;
//    SSP1CON1bits.CKP=1;
    break;
    }while(1);
}


///////   Interrupts I2C handler
void ssp_handlerB(void)
{
    unsigned int i, stat;
    unsigned char data;

        if (SSPOV == 1)
        {
            SSPOV = 0;  //clear overflow
            data = SSPBUF;
        }
        else
        {
            stat = SSPSTAT;
            stat = stat & 0b00101101;
            //find which state we are in
            //state 1 Master just wrote our address
            if ((stat ^ 0b00001001) == 0) //S=1 && RW==0 && DA==0 && BF==1
            {
//                for (i=0; i<BUF_SIZE; ++i) //clear buffer
//                    buf[i] = 0;
                I2CByteCount=0;
                GotCommandFlag=0;
                //ReportAddress = SSPBUF; //read address so we know later if read or write
                data=SSPBUF;
            }
            //state 2 Master just wrote data
            else if ((stat ^ 0b00101001) == 0) //S=1 && RW==0 && DA==1 && BF==1
            {
                data = SSPBUF;
                if(GotCommandFlag)
                 {

                       if(I2CCommand==0)
                         {
                                 //  run mode
                               if(I2CByteCount==0)
                                 {

                                    CommandEnable= (data &1)==1 ? 1 : 0;
                                  }
                         }
                       else if(I2CCommand==1)
                        {  // Timer settings
                                   if(I2CByteCount==0)
                                      {
                                          BaudRate = data & 0x7;
                                          Settings.BaudRate=BaudRate;
                                      }
                       }
                        else if(I2CCommand==5)
                            {   // change I2C_channel
                                if(I2CByteCount==0)
                                 if(data > 0x2)
                                   if(data < 0x78)
                                       {
                                         // ok we are allowed to change Adress
                                         // will be only effective if it is save on flash
                                                 Settings.I2C_Address=data;
                                                 // tell main program to save and reload
                                        }
                            }
                         else if(I2CCommand==8)
                             {  //  Oscillator tune
                                      if(I2CByteCount==0)
                                         {
                                           if((data & 0x20)==0x20)
                                               data |= 0xC0;
                                              else
                                               data &= 0x1F;
                                            Settings.OscTune = data;

                                         }
                             }


                         else if(I2CCommand==9)
                             {  //  write flash
                                       if(I2CByteCount==0)
                                         {
                                             if(data != 0x55)
                                                    I2CByteCount=2;
                                         }
                                     else if(I2CByteCount==1)
                                         {
                                              if(data == 0xaa)
                                                SaveSettingsFlag=1;
                                         }
                             }
                    I2CByteCount++;
                 }
              else
               {
                   I2CCommand = data;
                   GotCommandFlag=1;
               }
        }
     		  //state 3  & 4 Master want to read data
              else if ((stat & 0b00001100) == 0b00001100)
{
                if((stat & 0b00100000)==0)
               {
                I2CByteCount=1;
                //ReportAddress = SSPBUF;
                data=SSPBUF;
               }

                  if(I2CCommand==0)
                         { // reconstruct Control
                             if(I2CByteCount==1)
                              {
                               data = CommandEnable ? 1 : 0;
                              }
                             else
                               data=0;
                       }
                  else if(I2CCommand ==1)
                   {// timer
                        if(I2CByteCount==1)
                        {
                             BaudRate= BaudRate & 0x7;
                             data = BaudRate;
                        }
                        else
                               data=0;
                   }
                else if(I2CCommand==8)
                    {  // OSC TUNE
                        if(I2CByteCount==1)
                             data = Settings.OscTune;
                       else
                          data= 0;
                  }
                 else if(I2CCommand==14)
                  {// ID
		       if(I2CByteCount==1)
                         {
                           data = IDTAGL;
                         }
                        else if(I2CByteCount ==2)
                          {
                           data =  IDTAGH;
                          }
                        else
                           data = 0;
                   }
                 else if(I2CCommand==15)
                  {// version
                        if(I2CByteCount==1)
                          {
                           data = MINOR_VERSION;
                          }
                        else if(I2CByteCount==2)
                          {
                           data = MAJOR_VERSION;
                          }
                        else
                           data=0;
                 }
                 else data=0;

                I2CByteCount++;
//                WCOL = 0;   //clear write collision flag
//                SSPBUF = data ; //data to send
                 I2CWrite(data);
            }
            //state 5 Master sends NACK to end message
//            else if ((stat ^ 0b00101000) == 0) //S=1 && RW==0 && DA==1 && BF==0
//            {
//                /dat = SSPBUF;
//            }
            else //undefined, clear buffer
            {   WCOL=0;
                data = SSPBUF;
            }
        }
    CKP = 1; //release the clk
    SSP1IF = 0; //clear interupt flag
    }



static void interrupt isr(void){
// check serial transmit Interrupt
if(IOCIE)
    if(IOCAFbits.IOCAF0)
    {
      OUT_ENABLE=1;         // enable transmission

        // OK The RPI is transmitting something
      // We are in transmit mode
      IOCAFbits.IOCAF0=0;
      IN_ENABLE=1;          // disable reception
      IN_ENABLE=1;
      asm("NOP");
      OUT_ENABLE=1;         // enable transmission
      OUT_ENABLE=1;
      TMR0= T0_TimeOut;     // Reset the time out timer with correct delay
      TMR0IF=0;             // clear the timer flag
      TMR0IE=1;             // enable interrupt for time out
    }


// do we have time out?
if(TMR0IE)
 if(TMR0IF)
  {
     // ok we got time out
         if(OUT_ENABLE)
            {
             // ok transmit time out
             OUT_ENABLE=0;       // disable transmission
             TMR0= T0_RcvDelay;  // Now set timer for 1 baud delay
             TMR0IF=0;
             TMR0IE=1;
           }
           else
           {
            // ok 0.5 ms out for receive enable
            IN_ENABLE=0;        // Enable Reception
            TMR0IE=0;       // Disable Timer0 interrupt
            TMR0IF=0;
          }
   }


  if(SSP1IF==1)
   {
     ssp_handlerB();
  }


}



void ReadBaudRate(void)
{


    unsigned char idx;
    unsigned char temp;

    idx = BaudRate & 0x7;  // read dip switch (RA0 & RA1)



    // ok change prescaler
    temp = BaudPrescaler[idx];

    if(temp!=T0_Prescaler)
    {
     T0_Prescaler= temp;
     OPTION_REG= T0_Prescaler;  //handy since we are just using Prescaler
                                // in OPTION_REG
    }

    // ok Time out
    temp = BaudTimeOut[idx];
    if(temp!= T0_TimeOut)
        T0_TimeOut= temp;

    // rcv delay
    temp = BaudRcvDelay[idx];
    if(temp!=T0_RcvDelay)
        T0_RcvDelay= temp;
}






// ************************************   MAIN **********************************************************
void main(void){

 	
	OSCCON		= 0b11110000;	// 32MHz
	OPTION_REG	= 0b00000011;	// pullups on, TMR0 @ Fosc/4/16 ( need to interrupt on every 80 clock 16*5)
	ANSELA		= 0b00000;	    // no analog pins
	LATA   	    = 0b00010000;   // DE and  RE disable  DE=0 RE=1
	WPUA		= 0b00000000;	// pull-up ON  RA0 
	TRISA		= 0b00001111;	// ALL INPUT except RA4 RA5 OUTPUT
    PORTA       = 0b00100000;
	VREGCON		= 0b00000010;	// lower power sleep please
    INTCON		= 0b00000000;	// no interrupt


T0_Prescaler = T0_9600_PRESCALER;
T0_TimeOut= T0_9600_TIME_OUT;
T0_RcvDelay = T0_9600_RCV_DELAY;

//  A/D & FVR OFF
ADCON0=0;
FVRCON=0;



// get the I2C address from the eeprom
LoadSettings();

CommandEnable=0;
BaudRate= Settings.BaudRate;
ReadBaudRate();

// ok interrupt. Enable RA0 interrupt on change. (up and down).
// if IN_TXM toggle ,we are in transmit mode

IOCAP = 0b00000001;   // enable positive edge on RA0
IOCAN = 0b00000001;   // enable negative edge on RA0
IOCAF = 0;            // clear flag
// enable interrupt
IOCIE = 0;  // enable interrupt on change
GIE=1;      // enable general interrupt

I2CInit();

OUT_ENABLE=0;
IN_ENABLE=1;


// enable interrupt EDGE on RA0



   while(1){

     if(SaveSettingsFlag==1)   // Do we need to save settings in eeprom;
           {
                SSP1ADD  = Settings.I2C_Address << 1;
                SaveSettings();
           }
            ReadBaudRate();

          if(CommandEnable==0)
            {
                TMR0IE=0;
                TMR0IF=0;
              if(IOCIE)
                 IOCIE =0;
                OUT_ENABLE=0;
                IN_ENABLE=1;
            }
            else
            {
              if(IOCIE==0)
               {
                 IOCAF=0;
                 IOCIE=1;
               }
            }
     }
}
