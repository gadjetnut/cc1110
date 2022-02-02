/*******************************************************************************
  Filename:    dma_man_trigger.c

  Description: This example shows how to trigger the DMA manually and use
               DMA to move data from one area of RAM to another.

*******************************************************************************/

/*******************************************************************************
* INCLUDES
*/
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

#include <ioCCxx10_bitdef.h>
#include <hal_types.h>
#include <dma.h>


/*******************************************************************************
* CONSTANTS
*/

/* String length */
#define DATA_AMOUNT 16


/*******************************************************************************
* LOCAL VARIABLES
*/

/* The data that is to be copied to another place in memory. */
static char data[DATA_AMOUNT] = "DMA man trigger";

/* The aread where the data is to be copied to. */
static char copy[DATA_AMOUNT];

/* DMA configuration descriptor used for memory copy. */
static DMA_DESC dmaConfig0;


/*******************************************************************************
* LOCAL FUNCTIONS
*/


/*******************************************************************************
* @fn          main
*
* @brief       Sets up the DMA configuration struct, saves its address, arms the
*              DMA channel and triggers it manually. After the DMA transfer is
*              complete, the DMA channel 0 interrupt flag is cleared (it is
*              automatically set).
*
* @param       void
*
* @return      0
*******************************************************************************/
int main(void)
{
    /* Configure DMA channel 0. Settings:
     * SRCADDR: address of the data to be copied (increasing).
     * DESTADDR: address the data will be copied to (increasing).
     * VLEN: use LEN for transfer count.
     * LEN: equal to the number of bytes to be transferred.
     * WORDSIZE: each transfer should transfer one byte.
     * TMODE: block mode.
     * TRIG: let the DMA channel be triggered manually, i.e., by setting the
     *       [DMAREQ.DMAREQ0] bit.
     * SRCINC: increment by one byte.
     * DESTINC: increment by one byte.
     * IRQMASK: disable interrupts from this channel.
     * M8: 0, irrelevant since we use LEN for transfer count.
     * PRIORITY: high.
     */
    dmaConfig0.SRCADDRH  = ((uint16)data >> 8) & 0x00FF;
    dmaConfig0.SRCADDRL  = (uint16)data & 0x00FF;
    dmaConfig0.DESTADDRH = ((uint16)copy >> 8) & 0x00FF;
    dmaConfig0.DESTADDRL = (uint16)copy & 0x00FF;
    dmaConfig0.VLEN      = DMA_VLEN_USE_LEN;
    dmaConfig0.LENH      = (DATA_AMOUNT >> 8) & 0x00FF;
    dmaConfig0.LENL      = DATA_AMOUNT & 0x00FF;
    dmaConfig0.WORDSIZE  = DMA_WORDSIZE_BYTE;
    dmaConfig0.TMODE     = DMA_TMODE_BLOCK;
    dmaConfig0.TRIG      = DMA_TRIG_NONE;
    dmaConfig0.SRCINC    = DMA_SRCINC_1;
    dmaConfig0.DESTINC   = DMA_DESTINC_1;
    dmaConfig0.IRQMASK   = DMA_IRQMASK_DISABLE;
    dmaConfig0.M8        = DMA_M8_USE_8_BITS;
    dmaConfig0.PRIORITY  = DMA_PRI_HIGH;

    /* The DMA configuration data structure may reside at any location in
     * unified memory space, and the address location is passed to the DMA
     * through DMA0CFGH:DMA0CFGL.
     */
    DMA0CFGH = ((uint16)&dmaConfig0 >> 8) & 0x00FF;
    DMA0CFGL = (uint16)&dmaConfig0 & 0x00FF;

    /* Arm the DMA channel, so that a DMA trigger can initiate DMA writing,
    and apply 45 NOP’s to allow the DMA arming to actually take effect. */
    DMAARM |= DMAARM0;
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");asm("NOP");

    /* Trigger the DMA channel manually. */
    DMAREQ |= DMAREQ0;

    /* Wait for the DMA transfer to complete. */
    while ( !(DMAIRQ & DMAIRQ_DMAIF0) );

    /* By now, the transfer is completed, so the transfer count is reached.
     * The DMA channel 0 interrupt flag is then set, so we clear it here.
     */
    DMAIRQ &= ~DMAIRQ_DMAIF0;


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
