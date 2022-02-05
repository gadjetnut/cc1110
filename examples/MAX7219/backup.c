
/***********************************************************************************
  Filename:     spi_ex0_master.c

  Description:  This example uses a master to send data to a slave using SPI.

  Comments:     To execute this example, use spi_ex0_slave.c in IAR project
                spi0_slave_cc2510, as the slave. The slave's code must be
                executed before executing the master's code, since the slave
                is clocked by the Master. The bytes sent are simply numbers
                0x00 upto BUFFER_SIZE. Note that if BUFFER_SIZE is larger than
                0xFF, than the element 0xFF in the array will have the value 0x00
                and so on. When bytes up to BUFFER_SIZE are sent, the example will
                end and the green LED will be set on SMARTRF04EB.

                Note:
                Remember to use common ground for the Master and Slave!

***********************************************************************************/

/***********************************************************************************
* INCLUDES
*/
#include <stdint.h>
#include <cc1110.h>
#include "../../lib/cc1110-ext.h"
#include "../../lib/clk_mgmt.h"

/***********************************************************************************
* CONSTANTS
*/

// These values will give a baud rate of approx. 1.002930 Mbps for 26 MHz clock
#define SPI_BAUD_M  60
#define SPI_BAUD_E  15

// Define size of buffer and number of bytes to send
#define BUFFER_SIZE 12

#define MAX_DIN P1_7
#define MAX_CS P1_6
#define MAX_CLK P1_5
//MAX7219
#define REG_DECODE        0x09                        // "decode mode" register
#define REG_INTENSITY     0x0a                        // "intensity" register
#define REG_SCAN_LIMIT    0x0b                        // "scan limit" register
#define REG_SHUTDOWN      0x0c                        // "shutdown" register
#define REG_DISPLAY_TEST  0x0f                        // "display test" register

static const struct {
	char   ascii;
	char   segs;
} MAX7219_Font[] = {
  {'A',0b1110111},       
  {'B',0b0011111},       
  {'C',0b1001110},       
  {'D',0b0111101},       
  {'E',0b1001111},       
  {'F',0b1000111},       
  {'G',0b1011110},       
  {'H',0b0110111},       
  {'I',0b0110000},       
  {'J',0b0111100},       
  {'L',0b0001110},       
  {'N',0b1110110},       
  {'O',0b1111110},       
  {'P',0b1100111},       
  {'R',0b0000101},       
  {'S',0b1011011},       
  {'T',0b0001111},       
  {'U',0b0111110},       
  {'Y',0b0100111},       
  {'[',0b1001110},
  {']',0b1111000},
  {'_',0b0001000},       
  {'a',0b1110111},       
  {'b',0b0011111},       
  {'c',0b0001101},       
  {'d',0b0111101},       
  {'e',0b1001111},       
  {'f',0b1000111},       
  {'g',0b1011110},       
  {'h',0b0010111},       
  {'i',0b0010000},        
  {'j',0b0111100},       
  {'l',0b0001110},       
  {'n',0b0010101},       
  {'o',0b1111110},       
  {'p',0b1100111},       
  {'r',0b0000101},       
  {'s',0b1011011},       
  {'t',0b0001111},       
  {'u',0b0011100},       
  {'y',0b0100111},       
  {'-', 0x00},
  {' ', 0x00},
  {'0', 0x7e},
  {'1', 0x30},
  {'2', 0x6d},
  {'3', 0x79},
  {'4', 0x33},
  {'5', 0x5b},
  {'6', 0x5f},
  {'7', 0x70},
  {'8', 0x7f},
  {'9', 0x7b},
  {'\0', 0x00}
};

/***********************************************************************************
* LOCAL VARIABLES
*/

// Masters's transmit buffer
__xdata static char txBufferMaster[BUFFER_SIZE];

/***********************************************************************************
* LOCAL FUNCTIONS
*/

//Microsecond delay
void delayMicro(int usec){
  for (int x=0;x<usec/3;x++){ //usec
  }
  return;
}


static void MAX7219_SendByte (unsigned char dataout)
{
  char i;
  for (i=8; i>0; i--) {
    unsigned char mask = 1 << (i - 1);                // calculate bitmask
    MAX_CLK=0;                                           // bring CLK low
    delayMicro(20);
    if (dataout & mask){                               // output one data bit
      MAX_DIN=1;                                         //  "1"
      delayMicro(20);
      }
    else{                                              //  or
      MAX_DIN=0;                                         //  "0"
      delayMicro(20);
    }
    MAX_CLK=1;                                           // bring CLK high
	}
}

static void MAX7219_Write (unsigned char reg_number, unsigned char dataout)
{
  MAX_CS=1;                                             // take LOAD high to begin
  MAX7219_SendByte(reg_number);                       // write register number to MAX7219
  MAX7219_SendByte(dataout);                          // write data to MAX7219
  MAX_CS=0;                                             // take LOAD low to latch in data
  MAX_CS=1;                                             // take LOAD high to end                                      // take LOAD high to end
}

void MAX7219_ShutdownStart (void)
{
  MAX7219_Write(REG_SHUTDOWN, 0);                     // put MAX7219 into "shutdown" mode
}

void MAX7219_DisplayTestStart (void)
{
  MAX7219_Write(REG_DISPLAY_TEST, 1);                 // put MAX7219 into "display test" mode
}

void MAX7219_Clear (void)
{
  char i;
  for (i=0; i < 8; i++)
    MAX7219_Write(i, 0x00);                           // turn all segments off
}

void MAX7219_DisplayTestStop (void)
{
  MAX7219_Write(REG_DISPLAY_TEST, 0);                 // put MAX7219 into "normal" mode
}

void MAX7219_SetBrightness (char brightness)
{
  brightness &= 0x0f;                                 // mask off extra bits
  MAX7219_Write(REG_INTENSITY, brightness);           // set brightness
}

static unsigned char MAX7219_LookupCode (char character, uint8_t dp)
{
  char i;
  uint8_t d=0;
  if (dp) d=1;  
  if (character>=35 && character<=44) {
    character+=13;
    d=1;
  }
  for (i = 0; MAX7219_Font[i].ascii; i++)             // scan font table for ascii code
    if (character == MAX7219_Font[i].ascii){
      if (d){
        d=MAX7219_Font[i].segs;
        d |= (1<<7);
        return (d);                    // return segments code
        }
      else
        return MAX7219_Font[i].segs;                    // return segments code
    }
      
  return 0;                                           // code not found, return null (blank)
}

//digit = display position 0-7, charachter = what to display, dp = decimal place
void MAX7219_DisplayChar (char digit, char character, uint8_t dp)
{
 MAX7219_Write(digit, MAX7219_LookupCode(character, dp));
}

void MAX7219_ShutdownStop (void)
{
  MAX7219_Write(REG_SHUTDOWN, 1);                     // put MAX7219 into "normal" mode
}


void MAX7219_Init (void)
{
  
  P1SEL |= (0<<5);  //0=General purpose IO, 1=Peripheral; VAL<<PIN
  P1SEL |= (0<<6);  //0=General purpose IO, 1=Peripheral; VAL<<PIN
  P1SEL |= (0<<7);  //0=General purpose IO, 1=Peripheral; VAL<<PIN
  P1DIR |= (1<<5);  //0=Input;              1=Output; VAL<<PIN P1_5 CLK
  P1DIR |= (1<<6);  //0=Input;              1=Output; VAL<<PIN P1_6 LOAD(CS)
  P1DIR |= (1<<7);  //0=Input;              1=Output; VAL<<PIN P1_7 DIN
}

/***********************************************************************************
* @fn          main
*
* @brief       Send data to a single slave using SPI in Master mode
*
* @param       void
*
* @return      0
*/
int main(void)
{

    MAX7219_Init();
    //MAX7219_DisplayTestStart();
    //MAX7219_DisplayTestStop();
    //MAX7219_SetBrightness (0xff);
    MAX7219_Write(2, 0b0001111);  
    MAX7219_Write(3, 0b0001111);  
    MAX7219_Write(4, 0b0001111);  
    //MAX7219_Clear ();
    /*
    MAX7219_DisplayChar (0, '1', 0);
    MAX7219_DisplayChar (1, 'A', 0);
    MAX7219_DisplayChar (2, '3', 0);
    MAX7219_DisplayChar (3, 'B', 0);
    MAX7219_DisplayChar (4, '5', 0);
    MAX7219_DisplayChar (5, 'C', 0);
    MAX7219_DisplayChar (6, '7', 0);
    MAX7219_DisplayChar (7, 'D', 0);
*/
    while(1){}
    
}


/***********************************************************************************
  Copyright 2008 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
***********************************************************************************/

