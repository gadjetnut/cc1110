#include <stdint.h>
#include <string.h>
#include "../../lib/cc1110-ext.h"
#include "../../lib/radio.h"
#include "../../lib/clk_mgmt.h"
#include "../../lib/interrupt.h"
#include "../../lib/delay.h"
#include "../../lib/sdcc_dma.h"

// Baudrate = 57.6 kbps (U0BAUD.BAUD_M = 34, U0GCR.BAUD_E = 11)
#define UART_BAUD_M  34
#define UART_BAUD_E  11

// DMA descriptor for UART RX/TX
static DMA_DESC __xdata uartDmaRxTxCh[2];

volatile uint8_t __xdata radioPktBuffer[PACKET_LENGTH + 3];

void rf_IRQ(void) __interrupt RF_VECTOR{
    RFIF &= ~IRQ_DONE;        // Tx/Rx completed, clear interrupt flag
    S1CON &= ~0x03;           // Clear the general RFIF interrupt registers

    if (mode == RADIO_MODE_RX) {
        pktRcvdFlag = 1;
    }
    else {
        pktSentFlag = 1;
        RFST = RFST_SIDLE;      
    }
}

/***********************************************************************************
* @fn          uart0StartTxDmaChan
*
* @brief       Function which sets up a DMA channel for UART0 TX.
*
* @param       DMA_DESC *uartDmaTxDescr - pointer to DMA descriptor for UART TX
*              uint8_t uartDmaTxChan - DMA channel number for UART TX
*              uint8_t* uartTxBuf - pointer to allocated UART TX buffer
*              uint16_t uartTxBufSize - size of allocated UART TX buffer
*
* @return      0
*/
void uart0StartTxDmaChan( DMA_DESC *uartDmaTxDescr,
                          uint8_t uartDmaTxChan,
                          uint8_t* uartTxBuf,
                          uint16_t uartTxBufSize)
{

  // Set source/destination pointer (UART TX buffer address) for UART TX DMA channel,
  // and total number of DMA word transfer (according to UART TX buffer size).
  uartDmaTxDescr->SRCADDRH   = (uint16_t)(uartTxBuf+1)>>8;
  uartDmaTxDescr->SRCADDRL   = (uint16_t)(uartTxBuf+1);
  uartDmaTxDescr->DESTADDRH  = ((uint16_t)(&X_U0DBUF) >> 8) & 0x00FF;
  uartDmaTxDescr->DESTADDRL  = (uint16_t)(&X_U0DBUF) & 0x00FF;
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
    DMA0CFGH = (uint8_t)((uint16_t)uartDmaTxDescr>>8);
    DMA0CFGL = (uint8_t)((uint16_t)uartDmaTxDescr&0x00FF);
  } else {
    DMA1CFGH = (uint8_t)((uint16_t)uartDmaTxDescr>>8);
    DMA1CFGL = (uint8_t)((uint16_t)uartDmaTxDescr&0x00FF);
  }

  // Arm the relevant DMA channel for UART TX, and apply 45 NOP's
  // to allow the DMA configuration to load
  DMAARM = ((1 << uartDmaTxChan) & (BIT4 | BIT3 | BIT2 | BIT1 | BIT0));
  __asm nop __endasm;__asm nop __endasm;__asm nop __endasm;__asm nop __endasm;__asm nop __endasm;__asm nop __endasm;__asm nop __endasm;
  __asm nop __endasm;__asm nop __endasm;__asm nop __endasm;__asm nop __endasm;__asm nop __endasm;__asm nop __endasm;__asm nop __endasm;
  __asm nop __endasm;__asm nop __endasm;__asm nop __endasm;__asm nop __endasm;__asm nop __endasm;__asm nop __endasm;__asm nop __endasm;
  __asm nop __endasm;__asm nop __endasm;__asm nop __endasm;__asm nop __endasm;__asm nop __endasm;__asm nop __endasm;__asm nop __endasm;
  __asm nop __endasm;__asm nop __endasm;__asm nop __endasm;__asm nop __endasm;__asm nop __endasm;__asm nop __endasm;__asm nop __endasm;
  __asm nop __endasm;__asm nop __endasm;__asm nop __endasm;__asm nop __endasm;__asm nop __endasm;__asm nop __endasm;__asm nop __endasm;
  __asm nop __endasm;__asm nop __endasm;__asm nop __endasm;

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


int send_to_serial (void)
    {

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
    uart0StartTxDmaChan(&uartDmaRxTxCh[1], 1, radioPktBuffer+3, PACKET_LENGTH-2);
    //uart0StartTxDmaChan(&uartDmaRxTxCh[1], 1, "HELLO", 5);

}

void main(void){
    
    /* Initialize P1_3 for LED output */

    P1SEL &= ~(BIT3);
    P1DIR |= (BIT3);
    P1_3 = 1;

    mode = RADIO_MODE_RX;

    // Choose the crystal oscillator as the system clock
    SetMainClkSrc(CRYSTAL);
    
    // Select frequency and data rate from LCD menu, then configure the radio
    radioConfigure(DATA_RATE_1_CC1110, FREQUENCY_1_CC1110);
    
    // Set up the DMA to move packet data from buffer to radio
    dmaRadioSetup(RADIO_MODE_RX);
    
    // Configure interrupt for every time a packet is sent
    HAL_INT_ENABLE(INUM_RF, INT_ON);    // Enable RF general interrupt
    RFIM = IRQ_DONE;                    // Mask IRQ_DONE flag only
    INT_GLOBAL_ENABLE(INT_ON);          // Enable interrupts globally

    pktRcvdFlag=0;

    // Start receiving
    DMAARM = DMAARM_CHANNEL0;           // Arm DMA channel 0
    RFST   = STROBE_RX;                 // Switch radio to RX

    while (1) {
        
        // Poll for incoming packet delivered by radio + dma
        if (pktRcvdFlag) {
            pktRcvdFlag = 0;
            //delayms(100);
            send_to_serial();
            //memcpy(radioPktBuffer, 0x00, PACKET_LENGTH + 3);
            if (radioPktBuffer[3]=='a'){
                P1_3^=1;    
            }       
            DMAARM = DMAARM_CHANNEL0;
            RFST = RFST_SRX;
        }           
    }
}

