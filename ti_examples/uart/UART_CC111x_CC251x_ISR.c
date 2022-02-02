
/***********************************************************************************
  Filename:     UART_CC111x_CC251x_ISR.c

  Description:  This example sends/receives data on UART0, using interrupt method.

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


// Size of allocated UART RX/TX buffer (just an example)
#define SIZE_OF_UART_RX_BUFFER   50
#define SIZE_OF_UART_TX_BUFFER   SIZE_OF_UART_RX_BUFFER

#define UART_TST_CHAR_1  0xA5
#define UART_TST_CHAR_2  0xB5

// Test definitions
#define UART_TST_MODE_RX
//#define UART_TST_MODE_TX

// Baudrate = 57.6 kbps (U0BAUD.BAUD_M = 34, U0GCR.BAUD_E = 11)
#define UART_BAUD_M  34
#define UART_BAUD_E  11



/***********************************************************************************
* LOCAL VARIABLES
*/

// Buffer+index for UART RX/TX
static uint16 __xdata uartRxBuffer[SIZE_OF_UART_RX_BUFFER];
static uint16 __xdata uartTxBuffer[SIZE_OF_UART_TX_BUFFER];
static uint16 __xdata uartTxIndex;
static uint16 __xdata uartRxIndex;

// Variable for buffer indexing
static uint16 __xdata i;

// Variable for UART packet monitoring
static uint8 __xdata uartPktReceived = 1;


// Prototype for local functions
void uart0StartRxForIsr(void);
void uart0StartTxForIsr(void);



/***********************************************************************************
* @fn          main
*
* @brief       Send/receive data on UART0, using interrupt.
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
     // Use debugger to check received UART packet
    if(uartPktReceived)
    {
      uartPktReceived = 0;
      for (i = 0; i < SIZE_OF_UART_RX_BUFFER; i++)
      {
        uartRxBuffer[i] = 0;
      }
      uart0StartRxForIsr();
    }
    #else
    // Start UART TX when S1 pressed
    if ( !(P0 & BIT1) )
    {
      uart0StartTxForIsr();
    }
    #endif
  }

}




/***********************************************************************************
* LOCAL FUNCTIONS
*/

/***********************************************************************************
* @fn          uart0StartTxForIsr
*
* @brief       Function which starts/initiates the transmission of byte sequence
*              on UART0.
*
* @param       none
*
* @return      0
*/
void uart0StartTxForIsr(void)
{

  // Initialize the UART TX buffer indexes.
  uartTxIndex = 0;

  // Clear any pending UART TX Interrupt Flag (IRCON2.UTXxIF = 0, UxCSR.TX_BYTE = 0)
  UTX0IF = 0; U0CSR &= ~U0CSR_TX_BYTE;

  // Send very first UART byte
  U0DBUF = uartTxBuffer[uartTxIndex++];

  // Enable global interrupt (IEN0.EA = 1) and UART TX Interrupt (IEN2.UTXxIE = 1)
  EA = 1; IEN2 |= IEN2_UTX0IE;
}




/***********************************************************************************
* @fn          uart0StartRxForIsr
*
* @brief       Function which starts/initiates the receive of a byte sequence
*              on UART0.
*
* @param       none
*
* @return      0
*/
void uart0StartRxForIsr(void)
{

  // Initialize the UART RX buffer index
  uartRxIndex = 0;

  // Clear any pending UART RX Interrupt Flag (TCON.URXxIF = 0, UxCSR.RX_BYTE = 0)
  URX0IF = 0; U0CSR &= ~U0CSR_RX_BYTE;

  // Enable UART RX (UxCSR.RE = 1)
  U0CSR |= U0CSR_RE;

  // Enable global Interrupt (IEN0.EA = 1) and UART RX Interrupt (IEN0.URXxIE = 1)
  EA = 1; URX0IE = 1;
}




/***********************************************************************************
* @fn          UART0_TX_ISR
*
* @brief       Function which completes the UART transmit session, that is it
*              sends the rest of the UART0 packet.
*
* @param       none
*
* @return      0
*/
#pragma vector = UTX0_VECTOR
__interrupt void UART0_TX_ISR(void)
{

  // Clear UART0 TX Interrupt Flag (IRCON2.UTX0IF = 0)
  UTX0IF = 0;

  // If no UART byte left to transmit, stop this UART TX session
  if (uartTxIndex > SIZE_OF_UART_TX_BUFFER)
  {
    // Note:
    // In order to start another UART TX session the application just needs
    // to prepare the source buffer, and simply send the very first byte.
    uartTxIndex = 0; IEN2 &= ~IEN2_UTX0IE; return;
  }

  // Send next UART byte
  U0DBUF = uartTxBuffer[uartTxIndex++];
}





/***********************************************************************************
* @fn          UART0_TX_ISR
*
* @brief       Function which completes the UART receive session.
*
* @param       none
*
* @return      0
*/
#pragma vector = URX0_VECTOR
__interrupt void UART0_RX_ISR(void)
{

  // Clear UART0 RX Interrupt Flag (TCON.URX0IF = 0)
  URX0IF = 0;

  // Read UART0 RX buffer
  uartRxBuffer[uartRxIndex++] = U0DBUF;

  // If all UART data received, stop this UART RX session
  if (uartRxIndex >= SIZE_OF_UART_RX_BUFFER) {
    uartRxIndex = 0;
    uartPktReceived = 1;
  }

}




