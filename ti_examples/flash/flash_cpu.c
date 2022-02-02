/*******************************************************************************
  Filename:    flash_cpu.c

  Description: This example illustrates how to write data to flash memory using
               the CPU (rather than DMA). Writing data to flash with the CPU can
               not be done while executing code from flash, so the function that
               writes to flash must be copied to RAM and executed from there.

               The default system clock settings are used, so that the system
               clock frequency F is 13 MHz. This makes the flash write timing
               FWT 17.065. 17 = 0x11, which is the default value of [FWT.FWT].

               Note that one can not step into a function running from RAM while
               debugging. IAR might also fail to show updated contents of flash,
               but if the debugger is restarted, the data should be visible
               unless the option "Erase flash" is chosen. The contents data
               written to flash is read back to RAM to check if the flash write
               succeeded.

*******************************************************************************/

/*******************************************************************************
* INCLUDES
*/

#include <hal_types.h>
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


/*******************************************************************************
* CONSTANTS
*/

/* One whole page of flash memory is to be reserved, i.e., 1 KiB. */
#define PAGE_SIZE 1024

/* String length (exluding the terminal '\0'). */
#define DATA_AMOUNT 16

/* Enough bytes to store the write function in RAM. */
#define RAM_BUF_SIZE 400


/*******************************************************************************
* LOCAL VARIABLES
*/

/* Where in RAM the function to be run will reside. Must be big enough! */
static uint16 __xdata ramFuncAddr[RAM_BUF_SIZE];

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


/*******************************************************************************
* LOCAL FUNCTIONS
*/

// Prototype for local functions
void runFunctionFromRam(void (*func)(void), uint16 __xdata *ramAddr, uint16 funcLength);
void flashWriter();
void halFlashStartErase(void); // Implemented in assembly
void halFlashStartWrite(void); // Implemented in assembly



/*******************************************************************************
* @fn          runFunctionFromRam
*
* @brief       Copies another function from flash to RAM and executes the
*              function. Does check whether the space provided in RAM is enough,
*              this must be done prior to the call.
*
* @param       void (*func)(void) - address of function to be run from RAM.
*              uint8 __xdata *ramAddr - adress of function location in RAM.
*              uint16 funcLength - size of buffer to place function in [bytes].
*
* @return      void
*******************************************************************************/
void runFunctionFromRam(void (*func)(void), uint16 __xdata *ramAddr,
						uint16 funcLength)
{
    /* flashFuncAddr is a pointer to where in flash the function is. */
    uint16 __code *flashFuncAddr = (uint16 __code *)(uint16)func;

    /* f is a function pointer to the address in RAM where the function will be
     * placed.
     */
    VFPTR f = (VFPTR)(uint16)ramAddr;

    /* Copy the function from flash to RAM. */
    uint16 i;
    for (i = 0; i < funcLength; i++)
    {
        ramAddr[i] = flashFuncAddr[i];
    }

    /* Run function from RAM. */
    (*f)();

    return;
}


/*******************************************************************************
* @fn          flashWriter
*
* @brief       Writes contents of the array "data" to flash. Must be run from
*              RAM. Relies on eksternal variables. Setup of FCTL, FADDRH and
*              FADDRL msut be done prior to running this function.
*
* @param       void
*
* @return      void
*******************************************************************************/
void flashWriter(void)
{
    /* Disable interrupts. */
    EA = 0;

    /* Waiting for the flash controller to be ready. */
    while (FCTL & FCTL_BUSY);

    /* Setup of FCTL, FADDRH and FADDRL done in main().
     * Enabling flash write.
     */
    FCTL |= FCTL_WRITE;

    /* Write all the data to flash. */
    uint8 i = 0;
    while (i < DATA_AMOUNT)
    {
        /* Write two bytes to FWDATA. Flash write starts after second byte is
         * written to the register. The first byte written to FWDATA is the LSB
         * of the 16-bit word.
         */
        FWDATA = data[i++];
        FWDATA = data[i++];

        /* Wait for the flash write to complete. */
        while (FCTL & FCTL_SWBSY);
    }

    /* If desired, interrupts may again be enabled. */

    return;
}


/*******************************************************************************
* @fn          main
*
* @brief       Sets up the flash controller, erases the page where data will be
*              written to, calls the functions that will write data to flash and
*              reads this data back to RAM.
*
* @param       void
*
* @return      0
*******************************************************************************/
int main(void)
{
    /* Wait for the flash controller to be ready. */
    while (FCTL & FCTL_BUSY);

    /* Setup FADDRH, FADDRL. See the datasheet for flash address details. */
    FADDRH = (uint16)flashDataAddr >> 9;
    FADDRL = ((uint16)flashDataAddr >> 1) & ~0xFF00;

    /* Setup FWT. This is default setting, matches 13 MHz clock frequency. */
    FWT = 0x11;

    /* Erase the page that later will be written to. (2-byte alignment.) */
    halFlashStartErase();

    /* Wait for the erase operation to complete. */
    while (FCTL & FCTL_BUSY);

    /* Copy the flash write function to RAM and execute it. */
    runFunctionFromRam(flashWriter, ramFuncAddr, sizeof ramFuncAddr);

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
