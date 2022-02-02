
/***********************************************************************************
  Filename:     UART_CC111x_CC251x_polling.c

  Description:  This example sends/receives data on UART0, using polling method.

  Comments:     To execute this example, compile one SMARTRF04EB unit as transmitter,
                by activating the UART_TST_MODE_TX definition, then compile another
                unit as receiver, by activating the UART_TST_MODE_RX definition.
                Pressing S1 makes the transmitter send its allocated uartTxBuffer[]
                to the receiver. The transferred data will arrive at uartRxBuffer[]
                and can be verified using the debugger.

                For more detailed info about the UART functionality, please study
                the design note DN112_Using_UART_in_CC111xFx_CC243x_and_CC251xFx.

                Note:
                Once the UART receiver has been enabled (U0CSR.RE = 1) it will
                automatically trigger data reception as soon the Start bit is
                detected. The transmitter I/O should therefor be configured
                before the receiver, such that potential pin toggling (during
                I/O configuration of the transmitter) does not trigger data
                reception at the receiver side. Also, remember to use common
                ground for the TX unit and RX unit!

***********************************************************************************/


/***********************************************************************************
* INCLUDES
*/

#include <hal_types.h>
#include <hal_defs.h>
#include <hal_cc8051.h>
#include <ioCCxx10_bitdef.h>

#if (chip == 2510)
#include <ioCC2510.h>
#endif
#if (chip == 1110)
#include <ioCC1110.h>
#endif
#if (chip == 2511)
#include <ioCC2511.h>
#endif
#if (chip == 1111)
#include <ioCC1111.h>
#endif


/***********************************************************************************
* CONSTANTS
*/


// Define size of allocated UART RX/TX buffer (just an example)
#define SIZE_OF_UART_RX_BUFFER   50
#define SIZE_OF_UART_TX_BUFFER   SIZE_OF_UART_RX_BUFFER

#define UART_TST_CHAR_1  0xA5
#define UART_TST_CHAR_2  0xB5

// Test definitions
//#define UART_TST_MODE_RX
#define UART_TST_MODE_TX

// Baudrate = 57.6 kbps (U0BAUD.BAUD_M = 34, U0GCR.BAUD_E = 11)
#define UART_BAUD_M  59
#define UART_BAUD_E  12


/***********************************************************************************
* LOCAL VARIABLES
*/

// Buffer for UART RX/TX
static uint16 __xdata uartRxBuffer[SIZE_OF_UART_RX_BUFFER];
static uint16 __xdata uartTxBuffer[SIZE_OF_UART_TX_BUFFER];

// Variable for buffer indexing
static uint16 __xdata i;

// Prototype for local functions
void uart0Send(uint16* uartTxBuf, uint16 uartTxBufLength);
void uart0Receive(uint16* uartRxBuf, uint16 uartRxBufLength);


/***********************************************************************************
* @fn          main
*
* @brief       Send/receive data on UART0, using polling.
*
* @param       void
*
* @return      0
*/
int main (void)
{

  /***************************************************************************
   * Setup test stuff
   */

  // Initialize P0_1 for SRF04EB S1 button
  P0SEL &= ~BIT1;
  P0DIR &= ~BIT1;
  P0INP |= BIT1;


  // Initialise the allocated UART buffers:
  for (i = 0; i < SIZE_OF_UART_RX_BUFFER; i++)
  {
    uartTxBuffer[i] = (i%2) ? UART_TST_CHAR_1 : UART_TST_CHAR_2;
    uartRxBuffer[i] = 0;
  }



  /***************************************************************************
   * Setup I/O ports
   *
   * Port and pins used by USART0 operating in UART-mode are
   * RX     : P0_2
   * TX     : P0_3
   * CT/CTS : P0_4
   * RT/RTS : P0_5
   *
   * These pins can be set to function as peripheral I/O to be be used by UART0.
   * The TX pin on the transmitter must be connected to the RX pin on the receiver.
   * If enabling hardware flow control (U0UCR.FLOW = 1) the CT/CTS (Clear-To-Send)
   * on the transmitter must be connected to the RS/RTS (Ready-To-Send) pin on the
   * receiver.
   */

  // Configure USART0 for Alternative 1 => Port P0 (PERCFG.U0CFG = 0)
  // To avoid potential I/O conflict with USART1:
  // configure USART1 for Alternative 2 => Port P1 (PERCFG.U1CFG = 1)
  PERCFG = (PERCFG & ~PERCFG_U0CFG) | PERCFG_U1CFG;

  // Configure relevant Port P0 pins for peripheral function:
  // P0SEL.SELP0_2/3/4/5 = 1 => RX = P0_2, TX = P0_3, CT = P0_4, RT = P0_5
  P0SEL |= BIT5 | BIT4 | BIT3 | BIT2;



  /***************************************************************************
   * Configure UART
   *
   * The system clock source used is the HS XOSC at 26 MHz speed.
   */

  // Set system clock source to 26 Mhz XSOSC to support maximum transfer speed,
  // ref. [clk]=>[clk_xosc.c]
  SLEEP &= ~SLEEP_OSC_PD;
  while( !(SLEEP & SLEEP_XOSC_S) );
  CLKCON = (CLKCON & ~(CLKCON_CLKSPD | CLKCON_OSC)) | CLKSPD_DIV_1;
  while (CLKCON & CLKCON_OSC);
  SLEEP |= SLEEP_OSC_PD;


  // Initialise bitrate = 57.6 kbps (U0BAUD.BAUD_M = 34, U0GCR.BAUD_E = 11)
  U0BAUD = UART_BAUD_M;
  U0GCR = (U0GCR&~U0GCR_BAUD_E) | UART_BAUD_E;




  // Initialise UART protocol (start/stop bit, data bits, parity, etc.):

  // USART mode = UART (U0CSR.MODE = 1)
  U0CSR |= U0CSR_MODE;

  // Start bit level = low => Idle level = high  (U0UCR.START = 0)
  U0UCR &= ~U0UCR_START;

  // Stop bit level = high (U0UCR.STOP = 1)
  U0UCR |= U0UCR_STOP;

  // Number of stop bits = 1 (U0UCR.SPB = 0)
  U0UCR &= ~U0UCR_SPB;

  // Parity = disabled (U0UCR.PARITY = 0)
  U0UCR &= ~U0UCR_PARITY;

  // 9-bit data enable = 8 bits transfer (U0UCR.BIT9 = 0)
  U0UCR &= ~U0UCR_BIT9;

  // Level of bit 9 = 0 (U0UCR.D9 = 0), used when U0UCR.BIT9 = 1
  // Level of bit 9 = 1 (U0UCR.D9 = 1), used when U0UCR.BIT9 = 1
  // Parity = Even (U0UCR.D9 = 0), used when U0UCR.PARITY = 1
  // Parity = Odd (U0UCR.D9 = 1), used when U0UCR.PARITY = 1
  U0UCR &= ~U0UCR_D9;

  // Flow control = disabled (U0UCR.FLOW = 0)
  U0UCR &= ~U0UCR_FLOW;

  // Bit order = LSB first (U0GCR.ORDER = 0)
  U0GCR &= ~U0GCR_ORDER;



  /***************************************************************************
   * Transfer UART data
   */
  while(1)
  {

    #ifdef UART_TST_MODE_RX
    for (i = 0; i < SIZE_OF_UART_RX_BUFFER; i++)
    {
      uartRxBuffer[i] = 0;
    }
    uart0Receive((uint16*)uartRxBuffer, SIZE_OF_UART_RX_BUFFER);
    #else
    // If SRF04 S1 button pressed
    if ( !(P0 & BIT1) )
    {
        uart0Send((uint16*)uartTxBuffer, SIZE_OF_UART_TX_BUFFER);
    }
    #endif
  }

  return 0;

}





/***********************************************************************************
* LOCAL FUNCTIONS
*/


/***********************************************************************************
* @fn          uart0Send
*
* @brief       Function which sends a requested number of bytes on UART0
*
* @param       uint16* uartTxBuf - address of allocated UART TX buffer
*              uint16 uartTxBufLength - size of allocated UART TX buffer
*
* @return      0
*/

void uart0Send(uint16* uartTxBuf, uint16 uartTxBufLength)
{
  uint16 uartTxIndex;

  // Clear any pending TX interrupt request (set U0CSR.TX_BYTE = 0)
  U0CSR &= ~U0CSR_TX_BYTE;

  // Loop: send each UART0 sample on the UART0 TX line
  for (uartTxIndex = 0; uartTxIndex < uartTxBufLength; uartTxIndex++)
  {
    U0DBUF = uartTxBuf[uartTxIndex];
    while(! (U0CSR&U0CSR_TX_BYTE) );
    U0CSR &= ~U0CSR_TX_BYTE;
  }

}



/***********************************************************************************
* @fn          uart0Receive
*
* @brief       Function which receives a requested number of bytes on UART0
*
* @param       uint16* uartRxBuf - address of allocated UART RX buffer
*              uint16 uartRxBufLength - size of allocated UART RX buffer
*
* @return      0
*/
void uart0Receive(uint16* uartRxBuf, uint16 uartRxBufLength)
{
  uint16 uartRxIndex;

  // Enable UART0 RX (U0CSR.RE = 1)
  U0CSR |= U0CSR_RE;

  // Clear any pending RX interrupt request (set U0CSR.RX_BYTE = 0)
  U0CSR &= ~U0CSR_RX_BYTE;

  // Loop: receive each UART0 sample from the UART0 RX line
  for (uartRxIndex = 0; uartRxIndex < uartRxBufLength; uartRxIndex++)
  {

    // Wait until data received (U0CSR.RX_BYTE = 1)
    while( !(U0CSR&U0CSR_RX_BYTE) );

    // Read UART0 RX buffer
    uartRxBuf[uartRxIndex] = U0DBUF;
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
