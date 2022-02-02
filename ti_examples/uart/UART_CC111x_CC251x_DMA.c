
/***********************************************************************************
  Filename:     UART_CC111x_CC251x_DMA.c

  Description:  This example sends/receives data on UART0, using DMA method.

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
#include <dma.h>

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
#define SIZE_OF_UART_RX_BUFFER   200
#define SIZE_OF_UART_TX_BUFFER   SIZE_OF_UART_RX_BUFFER

// UART test characters
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
static uint8 __xdata uartRxBuffer[SIZE_OF_UART_RX_BUFFER];
static uint8 __xdata uartTxBuffer[SIZE_OF_UART_TX_BUFFER];
static uint16 __xdata uartTxIndex;
static uint16 __xdata uartRxIndex;

// Variable for buffer indexing
static uint16 __xdata i;

// Variable for UART packet monitoring
static uint8 __xdata uartPktReceived = 1;



// DMA descriptor for UART RX/TX
static DMA_DESC __xdata uartDmaRxTxCh[2];


// Prototype for local functions
void uart0StartRxDmaChan( DMA_DESC *uartDmaRxDescr,
                          uint8 uartDmaRxChan,
                          uint8* uartRxBuf,
                          uint16 uartRxBufSize);
void uart0StartTxDmaChan( DMA_DESC *uartDmaTxDescr,
                          uint8 uartDmaTxChan,
                          uint8* uartTxBuf,
                          uint16 uartTxBufSize);




/***********************************************************************************
* @fn          main
*
* @brief       Send/receive data on UART0, using DMA.
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
      uart0StartRxDmaChan(&uartDmaRxTxCh[0], 0, uartRxBuffer, SIZE_OF_UART_RX_BUFFER);
    }
    #else
    // Start UART TX when S1 pressed
    if ( !(P0 & BIT1) )
    {
      uart0StartTxDmaChan(&uartDmaRxTxCh[1], 1, uartTxBuffer, SIZE_OF_UART_TX_BUFFER);
    }
    #endif
  }



  return 0;

}





/***********************************************************************************
* LOCAL FUNCTIONS
*/



/***********************************************************************************
* @fn          uart0StartTxDmaChan
*
* @brief       Function which sets up a DMA channel for UART0 TX.
*
* @param       DMA_DESC *uartDmaTxDescr - pointer to DMA descriptor for UART TX
*              uint8 uartDmaTxChan - DMA channel number for UART TX
*              uint8* uartTxBuf - pointer to allocated UART TX buffer
*              uint16 uartTxBufSize - size of allocated UART TX buffer
*
* @return      0
*/
void uart0StartTxDmaChan( DMA_DESC *uartDmaTxDescr,
                          uint8 uartDmaTxChan,
                          uint8* uartTxBuf,
                          uint16 uartTxBufSize)
{

  // Set source/destination pointer (UART TX buffer address) for UART TX DMA channel,
  // and total number of DMA word transfer (according to UART TX buffer size).
  uartDmaTxDescr->SRCADDRH   = (uint16)(uartTxBuf+1)>>8;
  uartDmaTxDescr->SRCADDRL   = (uint16)(uartTxBuf+1);
  uartDmaTxDescr->DESTADDRH  = ((uint16)(&X_U0DBUF) >> 8) & 0x00FF;
  uartDmaTxDescr->DESTADDRL  = (uint16)(&X_U0DBUF) & 0x00FF;
  uartDmaTxDescr->LENH       = ((uartTxBufSize-1)>>8)&0xFF;
  uartDmaTxDescr->LENL       = (uartTxBufSize-1)&0xFF;
  uartDmaTxDescr->VLEN       = DMA_VLEN_FIXED;  // Use fixed length DMA transfer count

  // Perform 1-byte transfers
  uartDmaTxDescr->WORDSIZE   = DMA_WORDSIZE_BYTE;

  // Transfer a single word after each DMA trigger
  uartDmaTxDescr->TMODE      = DMA_TMODE_SINGLE;

  // DMA word trigger = USARTx TX complete
  uartDmaTxDescr->TRIG       = DMA_TRIG_UTX0;

  uartDmaTxDescr->SRCINC     = DMA_SRCINC_1;       // Increment source pointer by 1 word
                                               // address after each transfer.
  uartDmaTxDescr->DESTINC    = DMA_DESTINC_0;      // Do not increment destination pointer:
                                               // points to USART UxDBUF register.
  uartDmaTxDescr->IRQMASK    = DMA_IRQMASK_ENABLE; // Enable DMA interrupt to the CPU
  uartDmaTxDescr->M8         = DMA_M8_USE_8_BITS;  // Use all 8 bits for transfer count
  uartDmaTxDescr->PRIORITY   = DMA_PRI_LOW;        // DMA memory access has low priority

  // Link DMA descriptor with its corresponding DMA configuration register.
  if (uartDmaTxChan < 1)
  {
    DMA0CFGH = (uint8)((uint16)uartDmaTxDescr>>8);
    DMA0CFGL = (uint8)((uint16)uartDmaTxDescr&0x00FF);
  } else {
    DMA1CFGH = (uint8)((uint16)uartDmaTxDescr>>8);
    DMA1CFGL = (uint8)((uint16)uartDmaTxDescr&0x00FF);
  }

  // Arm the relevant DMA channel for UART TX, and apply 45 NOP's
  // to allow the DMA configuration to load
  DMAARM = ((1 << uartDmaTxChan) & (BIT4 | BIT3 | BIT2 | BIT1 | BIT0));
  asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
  asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
  asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
  asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
  asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
  asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
  asm("NOP");asm("NOP");asm("NOP");

  // Enable the DMA interrupt (IEN1.DMAIE = IEN0.EA = 1),
  // and clear potential pending DMA interrupt requests (IRCON.DMAIF = 0).
  EA = 1; DMAIE = 1; DMAIF = 0;

  // Send the very first UART byte to trigger a UART TX session:
  U0DBUF = uartTxBuf[0];


  // At this point the UART peripheral generates a DMA trigger each time it has
  // transmitted a byte, leading to a DMA transfer from the allocated source buffer
  // to the UxDBUF register. Once the DMA controller has completed the defined
  // range of transfers, the CPU vectors its execution to the DMA ISR.
}




/***********************************************************************************
* @fn          uart0StartRxForIsr
*
* @brief       Function which sets up a DMA channel for UART0 RX.
*
* @param       DMA_DESC *uartDmaRxDescr - pointer to DMA descriptor for UART RX
*              uint8 uartDmaRxChan - DMA channel number for UART RX
*              uint8* uartRxBuf - pointer to allocated UART RX buffer
*              uint16 uartRxBufSize - size of allocated UART RX buffer
*
* @return      0
*/
void uart0StartRxDmaChan( DMA_DESC *uartDmaRxDescr,
                          uint8 uartDmaRxChan,
                          uint8* uartRxBuf,
                          uint16 uartRxBufSize)
{

  // Set source/destination pointer (UART RX buffer address) for UART RX DMA channel,
  // and total number of DMA word transfer (according to UART RX buffer size).
  uartDmaRxDescr->DESTADDRH  = (uint16)uartRxBuf>>8;
  uartDmaRxDescr->DESTADDRL  = (uint16)uartRxBuf;
  uartDmaRxDescr->SRCADDRH   = ((uint16)(&X_U0DBUF) >> 8) & 0x00FF;
  uartDmaRxDescr->SRCADDRL   = (uint16)(&X_U0DBUF) & 0x00FF;
  uartDmaRxDescr->LENH       = (uartRxBufSize>>8)&0xFF;
  uartDmaRxDescr->LENL       = uartRxBufSize&0xFF;

  uartDmaRxDescr->VLEN       = DMA_VLEN_FIXED;  // Use fixed length DMA transfer count

  // Perform 1-byte transfers
  uartDmaRxDescr->WORDSIZE   = DMA_WORDSIZE_BYTE;

  // Transfer a single word after each DMA trigger
  uartDmaRxDescr->TMODE      = DMA_TMODE_SINGLE;

  // DMA word trigger = USART0 RX complete
  uartDmaRxDescr->TRIG       = DMA_TRIG_URX0;

  uartDmaRxDescr->SRCINC     = DMA_SRCINC_0;       // Do not increment source pointer;
                                               // points to USART UxDBUF register.
  uartDmaRxDescr->DESTINC    = DMA_DESTINC_1;      // Increment destination pointer by
                                               // 1 word address after each transfer.
  uartDmaRxDescr->IRQMASK    = DMA_IRQMASK_ENABLE; // Enable DMA interrupt to the CPU
  uartDmaRxDescr->M8         = DMA_M8_USE_8_BITS;  // Use all 8 bits for transfer count
  uartDmaRxDescr->PRIORITY   = DMA_PRI_LOW;        // DMA memory access has low priority

  // Link DMA descriptor with its corresponding DMA configuration register.
  if (uartDmaRxChan < 1)
  {
    DMA0CFGH = (uint8)((uint16)uartDmaRxDescr>>8);
    DMA0CFGL = (uint8)((uint16)uartDmaRxDescr&0x00FF);
  } else {
    DMA1CFGH = (uint8)((uint16)uartDmaRxDescr>>8);
    DMA1CFGL = (uint8)((uint16)uartDmaRxDescr&0x00FF);
  }
  // Arm the relevant DMA channel for I2S RX, and apply 45 NOP's
  // to allow the DMA configuration to load
  DMAARM = ((1 << uartDmaRxChan) & (BIT4 | BIT3 | BIT2 | BIT1 | BIT0));
  asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
  asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
  asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
  asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
  asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
  asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
  asm("NOP");asm("NOP");asm("NOP");

  // Enable the DMA interrupt (IEN1.DMAIE = IEN0.EA = 1),
  // and clear potential pending DMA interrupt requests (IRCON.DMAIF = 0)
  EA = 1; DMAIE = 1; DMAIF = 0;

  // Enable UARTx RX, clear any pending RX interrupt request and permit data flow:
  // Enable UART0 RX (U0CSR.RE = 1)
  U0CSR |= U0CSR_RE;

  // At this point the UART peripheral generates a DMA trigger each time it
  // has received a byte, leading to a DMA transfer from the UxDBUF register
  // to the allocated target buffer. Once the DMA controller has completed the
  // defined range of transfers, the CPU vectors its execution to the DMA ISR.
}





/***********************************************************************************
* @fn          DMA_ISR
*
* @brief       DMA Interrupt Service Routine, called when DMA has finished
*              transferring one packet/buffer between memory and UxDBUF.
*
* @param       none
*
* @return      0
*/
#pragma vector = DMA_VECTOR
__interrupt void DMA_ISR(void)
{

  // Clear the main DMA interrupt Request Flag (IRCON.DMAIF = 0)
  DMAIF = 0;

  // Start a new UART RX session on DMA channel 1:
  if (DMAIRQ & DMAIRQ_DMAIF0)
  {
    // Indicate UART packet received (monitored by main-loop for data integrity/error check
    uartPktReceived = 1;

    // Clear DMA Channel 0 Interrupt Request Flag (DMAIRQ.DMAIF0 = 0)
    DMAIRQ &= ~DMAIRQ_DMAIF0;

    // Re-arm DMA Channel 0 (DMAARM.DMAARM0 = 1)
    DMAARM |= DMAARM0;
  }

  // Start a new UART TX session on DMA channel 1:
  if (DMAIRQ & DMAIRQ_DMAIF1)
  {
    // Clear DMA Channel 1 Interrupt Request Flag (DMAIRQ.DMAIF1 = 0)
    DMAIRQ &= ~DMAIRQ_DMAIF1;

    /* In this particular software example a new UART/DMA TX session must
     * be initialized outside this DMA ISR. However, it would also be possible
     * to use this DMA ISR to start a new UART/DMA TX session by simply
     * re-arming the DMA channel and sending the first UART TX byte.
     */
  }

}

