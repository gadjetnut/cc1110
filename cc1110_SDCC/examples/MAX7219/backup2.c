
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
#define BUFFER_SIZE 0x05

/***********************************************************************************
* LOCAL VARIABLES
*/

// Masters's transmit buffer
static uint8 txBufferMaster[BUFFER_SIZE];

/***********************************************************************************
* LOCAL FUNCTIONS
*/


/***********************************************************************************
* @fn          main
*
* @brief       Send data to a single slave using SPI in Master mode
*
* @param       void
*
* @return      0
*/

//Microsecond delay
void delayMicro(int usec){
  for (int x=0;x<usec/3;x++){ //usec
  }
  return;
}


static void MAX7219_Write (unsigned char reg_number, unsigned char dataout)
{
  P0_4 = 0;
    // Write byte to USART0 buffer (transmit data)
  U0DBUF = reg_number;

  // Check if byte is transmitted
  while(!(U0CSR & U0CSR_TX_BYTE));

  // Clear transmit byte status
  U0CSR &= ~U0CSR_TX_BYTE;
    // Write byte to USART0 buffer (transmit data)

  U0DBUF = dataout;

  // Check if byte is transmitted
  while(!(U0CSR & U0CSR_TX_BYTE));

  // Clear transmit byte status
  U0CSR &= ~U0CSR_TX_BYTE;
    
  P0_4 = 0;
  }

//Millisecond delay
void delayms(int msec){
  for (int x=0;x<msec;x++){ //msec
    for (int y=0;y<500;y++){ //usec
      }
  }
  return;
}

int main(void)
{

    /***************************************************************************
     * Setup test stuff
     */
    // Initialize P1_1/3 for SRF04EB LED1/3
    P1SEL &= ~(BIT3 | BIT0);
    P1_0 = 1; P1_3 = 0;
    P1DIR |= (BIT3 | BIT0);

    /***************************************************************************
     * Setup I/O ports
     *
     * Port and pins used by USART0 operating in SPI-mode are
     * MISO (MI): P0_2
     * MOSI (MO): P0_3 
     * SSN (SS) : P0_4
     * SCK (C)  : P0_5
     *
     * These pins can be set to function as peripheral I/O to be be used by
     * USART0 SPI. Note however, when SPI is in master mode, only MOSI, MISO,
     * and SCK should be configured as peripheral I/O's. If the external
     * slave device requires a slave select signal (SSN), then the master
     * can control the external SSN by using one of its GPIO pin as output.
     */

    // Configure USART0 for Alternative 1 => Port P0 (PERCFG.U0CFG = 0)
    // To avoid potential I/O conflict with USART1:
    // configure USART1 for Alternative 2 => Port P1 (PERCFG.U1CFG = 1)
    PERCFG = (PERCFG & ~PERCFG_U0CFG) | PERCFG_U1CFG;

    // Give priority to USART 0 over USART 1 for port 0 pins
    P2DIR = (P2DIR & ~P2DIR_PRIP0) | P2DIR_PRIP0_0;

    // Set pins 2, 3 and 5 as peripheral I/O and pin 4 as GPIO output
    //P0SEL = (P0SEL & ~BIT4) | BIT5 | BIT3 | BIT2;
    //P0DIR |= BIT4;

   P0SEL |= (1<<3) | (1<<4) | (1<<5);  //0=General purpose IO, 1=Peripheral; VAL<<PIN
   P0DIR |= (1<<3) | (1<<4) | (1<<5);  //0=Input;              1=Output; VAL<<PIN



    /***************************************************************************
     * Configure SPI
     */

    // Set system clock source to 26 Mhz XSOSC to support maximum transfer speed,
    // ref. [clk]=>[clk_xosc.c]
    SLEEP &= ~SLEEP_OSC_PD;
    while( !(SLEEP & SLEEP_XOSC_S) );
    CLKCON = (CLKCON & ~(CLKCON_CLKSPD | CLKCON_OSC)) | CLKSPD_DIV_1;
    while (CLKCON & CLKCON_OSC);
    SLEEP |= SLEEP_OSC_PD;

    // Set USART to SPI mode and Master mode
    U0CSR &= ~(U0CSR_MODE | U0CSR_SLAVE);

    // Set:
    // - mantissa value
    // - exponent value
    // - clock phase to be centered on first edge of SCK period
    // - negative clock polarity (SCK low when idle)
    // - bit order for transfers to LSB first
    U0BAUD = SPI_BAUD_M;
    U0GCR = (U0GCR & ~(U0GCR_BAUD_E | U0GCR_CPOL | U0GCR_CPHA | U0GCR_ORDER))
        | SPI_BAUD_E;
    U0GCR |= (1<<5);
    U0GCR |= (1<<6);
    U0GCR |= (1<<7);

    /***************************************************************************
     * Transfer data
     */


    MAX7219_Write(0x0f, 0x01);                 // put MAX7219 into "display test" mode
    MAX7219_Write(0x0c, 0x01);                 // put MAX7219 into "display test" mode
    MAX7219_Write(0x0a, 0x04);                 // put MAX7219 into "display test" mode
    MAX7219_Write(0x02, 0b1110111);            // put MAX7219 into "display test" mode

    

    while (1){ 
      P1_3^=1;
      delayms(1000);
      //MAX7219_Write(2, 0b1110111);                 // put MAX7219 into "display test" mode
      //MAX7219_Write(0xff, 0);                 // put MAX7219 into "display test" mode
      }
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
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

