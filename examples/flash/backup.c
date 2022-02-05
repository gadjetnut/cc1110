/*******************************************************************************
  Filename:    flash_dma.c

  Description: This example illustrates how to write data to flash memory using
               DMA (rather than the CPU). The DMA transfer method is the
               preferred way to write to the flash memory, since when using the
               DMA to write to flash, the code can be executed from within flash
               memory.

               The default system clock settings are used, so that the system
               clock frequency F is 13 MHz. This makes the flash write timing
               FWT 17.065. 17 = 0x11, which is the default value of FWT.FWT.

               When performing DMA flash write while executing code from within
               flash memory, the instruction that triggers the first DMA trigger
               event FLASH (TRIG[4:0]=10010) must be aligned on a 2-byte
               boundary. Therefore, the function halFlashStartWrite() is
               implemented in assembly.

               IAR might fail to show updated contents of flash, but if the
               debugger is restarted, the data should be visible unless the
               option "Erase flash" is chosen. The contents data written to
               flash is read back to RAM to check if the flash write succeeded.

*******************************************************************************/

/*******************************************************************************
* INCLUDES
*/

#include <stdint.h>
#include <string.h>
#include "../../lib/cc1110.h"
#include "../../lib/cc1110-ext.h"
#include "../../lib/clk_mgmt.h"
#include "../../lib/sdcc_dma.h"

/*******************************************************************************
* CONSTANTS
*/

/* One whole page of flash memory is to be reserved, i.e., 1 KiB. */
#define PAGE_SIZE 1024

/* String length (exludes the terminal '\0'). */
#define DATA_AMOUNT 16 + 1


/*******************************************************************************
* LOCAL VARIABLES
*/

/* The "string" to be written to flash ('\0' not included). */
static const char data[DATA_AMOUNT] = "Flash Controller";

// Baudrate = 57.6 kbps (U0BAUD.BAUD_M = 34, U0GCR.BAUD_E = 11) for serial output
#define UART_BAUD_M  34
#define UART_BAUD_E  11

// DMA descriptor for UART RX/TX
static DMA_DESC __xdata uartDmaRxTxCh[2];


/* String that is filled by reading from the data area that was written to in
 * flash. Can be used to debug the example, since the debugger may suggest that
 * nothing was written to flash.
 */
uint8 __xdata writeCheck[DATA_AMOUNT];

/* The area in flash where the string (data written to flash) will be placed.
 * If not placed at an even absolute location, the address might be odd (e.g.
 * 0x4401). Since flash is addressed in words, writing would start at one byte
 * before (or after) the address unless explicit action is made to avoid this.
 * Page 17 will be used, so the address is 0x4400 ( = 1024 * 17).
 */

//static const uint16_t flash_address = 0x4400;

//const char __code flashDataAddr[PAGE_SIZE] @ 0x4400;

//const __code char flashDataAddr[PAGE_SIZE] = (__code uint8_t *)0x4400;
static const uint16_t flash_address = 0x00;
__code unsigned char *flashDataAddr=0;

/* DMA configuration descriptor used for flash write. */
static DMA_DESC dmaConfig0;


/*******************************************************************************
* LOCAL FUNCTIONS
*/


uint8_t halFlashStartErase(uint8_t page) {
    // wait until flash controller is ready
    while (FCTL & FCTL_BUSY) {}

    // set up flash address FADDRH[5:1] set flash page
    FADDRH = page << 1;

    // configure flash controller for 26mhz clock
    FWT = 0x2A;

    // execute erase
    FCTL |= FCTL_ERASE;
    NOP();

    return 1;
}



void halFlashStartWrite(void){
    // trigger flash write. this generates a DMA trigger
    // this was moved to a separate function as sdcc does not
    // optimize functions that include asm code (!)
    __asm
    .even              // IMPORTANT: PLACE THIS ON A 2BYTE BOUNDARY!
    ORL _FCTL, #0x02;  // FCTL |=  FCTL_WRITE
    NOP
    __endasm;
}



/*******************************************************************************
* @fn          main
*
* @brief       Configures DMA channel 0, configures the flash controller,
*              erases the page to be written to. Then the DMA is armed and
*              triggered, which initiates the flash write. Upon completion of the
*              write, the DMA channel 0 interrupt flag is cleared. Finally,
*              the data written to flash is read back to RAM for debug purposes.
*
* @param       void
*
* @return      0
*******************************************************************************/

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
    uart0StartTxDmaChan(&uartDmaRxTxCh[1], 1, writeCheck, DATA_AMOUNT);
    return(0);
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
    /*
    delayms(500);
    strncpy(writeCheck,"Starting",8);
    send_to_serial();

    SLEEP &= ~OSC_PD_BIT;     // powering down all oscillators
    while(!XOSC_STABLE);      // waiting until the oscillator is stable
    NOP();
    CLKCON &= ~MAIN_OSC_BITS; // starting the Crystal Oscillator
    SLEEP |= OSC_PD_BIT;      // powering down the unused oscillator
*/
    delayms(500);

    /* Configure DMA channel 0. Settings:
     * SRCADDR: address of the data to be written to flash (increasing).
     * DESTADDR: the flash controller data register (fixed), so that the
     *     flash controller will write this data to flash.
     * VLEN: use LEN for transfer count.
     * LEN: equal to the number of bytes to be transferred.
     * WORDSIZE: each transfer should transfer one byte.
     * TMODE: should be set to single mode (see datasheet, DMA Flash Write).
     *     Each flash write complete will re-trigger the DMA channel.
     * TRIG: let the DMA channel be triggered by flash data write complete
           (trigger number 18). That is, the flash controller will trigger the
           DMA channel when the Flash Write Data register, FWDATA, is ready to
           receive new data.
     * SRCINC: increment by one byte.
     * DESTINC: fixed (always write to FWDATA).
     * IRQMASK: disable interrupts from this channel.
     * M8: 0, irrelevant since we use LEN for transfer count.
     * PRIORITY: high.
     */
    dmaConfig0.SRCADDRH  = ((uint16)data >> 8) & 0x00FF;
    dmaConfig0.SRCADDRL  = (uint16)data & 0x00FF;
    dmaConfig0.DESTADDRH = ((uint16)&X_FWDATA >> 8) & 0x00FF;
    dmaConfig0.DESTADDRL = (uint16)&X_FWDATA & 0x00FF;
    dmaConfig0.VLEN      = DMA_VLEN_USE_LEN;
    dmaConfig0.LENH      = (DATA_AMOUNT >> 8) & 0x00FF;
    dmaConfig0.LENL      = DATA_AMOUNT & 0x00FF;
    dmaConfig0.WORDSIZE  = DMA_WORDSIZE_BYTE;
    dmaConfig0.TMODE     = DMA_TMODE_SINGLE;
    dmaConfig0.TRIG      = DMA_TRIG_FLASH;
    dmaConfig0.SRCINC    = DMA_SRCINC_1;
    dmaConfig0.DESTINC   = DMA_DESTINC_0;
    dmaConfig0.IRQMASK   = DMA_IRQMASK_DISABLE;
    dmaConfig0.M8        = DMA_M8_USE_8_BITS;
    dmaConfig0.PRIORITY  = DMA_PRI_HIGH;

    /* The DMA configuration data structure may reside at any location in
     * unified memory space, and the address location is passed to the DMA
     * through DMA0CFGH:DMA0CFGL.
     */
    DMA0CFGH = ((uint16)&dmaConfig0 >> 8) & 0x00FF;
    DMA0CFGL = (uint16)&dmaConfig0 & 0x00FF;

    /* Waiting for the flash controller to be ready */
    while (FCTL & FCTL_BUSY);

    /* Configuring the flash controller. Setings:
     * FWT: 0x11 (default setting, matches 13 MHz clock frequency).
     * FADDRH:FADDRL: point to the area in flash to write to - flashDataAddr.
     */
    FWT = 0x11;
    flashDataAddr = (__code uint8_t *)flash_address;
    //FADDRH = (int)&flashDataAddr >> 9;
    //FADDRL = ((int)&flashDataAddr >> 1) & ~0xFF00;
    SET_WORD(FADDRH, FADDRL, ((uint16_t)flash_address)>>1);

    /* Erase the page that will be written to. */
    halFlashStartErase(flash_address);

    /* Wait for the erase operation to complete. */
    while (FCTL & FCTL_BUSY);

    /* Arm the DMA channel, so that a DMA trigger will initiate DMA writing. */
    DMAARM |= DMAARM0;

    /* Enable flash write. Generates a DMA trigger. Must be aligned on a 2-byte
     * boundary and is therefor implemented in assembly.
     */
    halFlashStartWrite();

    /* Wait for DMA transfer to complete. */
    while (!(DMAIRQ & DMAIRQ_DMAIF0));

    /* Wait until flash controller not busy. */
    while (FCTL & (FCTL_BUSY | FCTL_SWBSY));

    /* By now, the transfer is completed, so the transfer count is reached.
     * The DMA channel 0 interrupt flag is then set, so we clear it here.
     */
    DMAIRQ &= ~DMAIRQ_DMAIF0;

    /* Read from flash to check whether the write was successful. */
    flashDataAddr = (__code uint8_t *)flash_address;
    uint8 i;
    for (i = 0; i < DATA_AMOUNT; i++)
    {
        writeCheck[i] = flashDataAddr[i];
    }

    send_to_serial(); //send the data read from the flash to serial
    strncpy(writeCheck,"Done",4);
    send_to_serial();
    return 0;
}


/*******************************************************************************
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
*******************************************************************************/

