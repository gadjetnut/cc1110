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

/* One whole page of flash memory is to be reserved, i.e., 1 KiB. */
#define PAGE_SIZE 1024

/* String length (exluding the terminal '\0'). */
#define DATA_AMOUNT 16


/*******************************************************************************
* LOCAL VARIABLES
*/

/* The "string" to be written to flash ('\0' not included). */
static const char data[DATA_AMOUNT] = "Flash Controller";


/* String that is filled by reading from the data area that was written to in
 * flash. Can be used to debug the example, since the debugger may suggest that
 * nothing was written to flash.
 */
static char writeCheck[DATA_AMOUNT];

/* The area in flash where the string (data written to flash) will be placed.
 * If not placed at an even absolute location, the address might be odd (e.g.
 * 0x4401). Since flash is addressed in words, writing would start at one byte
 * before (or after) the address unless explicit action is made to avoid this.
 * Page 17 will be used, so the address is 0x4400 ( = 1024 * 17).
 */
__no_init const char __code flashDataAddr[PAGE_SIZE] @ 0x4400;

/* DMA configuration descriptor used for flash write. */
static DMA_DESC dmaConfig0;


/*******************************************************************************
* LOCAL FUNCTIONS
*/

// Prototype for local functions (implemented in assembly).
void halFlashStartErase(void);
void halFlashStartWrite(void);



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
int main(void)
{
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
    FADDRH = (int)flashDataAddr >> 9;
    FADDRL = ((int)flashDataAddr >> 1) & ~0xFF00;

    /* Erase the page that will be written to. */
    halFlashStartErase();

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
    uint8 i;
    for (i = 0; i < DATA_AMOUNT; i++)
    {
        writeCheck[i] = flashDataAddr[i];
    }

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
