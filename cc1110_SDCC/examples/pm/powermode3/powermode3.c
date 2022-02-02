
/***********************************************************************************
  Filename:     powermode3.c

  Description:  This software example shows how to properly enter Power Mode 3
                based on the CC111xFx/CC251xFx Errata Note, and then exit upon Port 0
                Interrupt. The SRF04EB LED1 is cleared before entering Power Mode 3,
                and then set by the Port 0 ISR. Hence, the SRF04EB LED1 is ON in
                Active Mode and OFF in Power Mode 3.

                Note that the system clock source must be HS RCSOC before
                entering Power Mode 3!

***********************************************************************************/

/***********************************************************************************
* INCLUDES
*/
#include <stdint.h>
#include "../../../lib/cc1110.h"
#include "../../../lib/cc1110-ext.h"


/***********************************************************************************
* CONSTANTS
*/

// Wait time in Active mode
#define ACT_MODE_TIME  10000


/***********************************************************************************
* LOCAL VARIABLES
*/

// Variable for active mode duration
static uint32_t __xdata activeModeCnt = 0;

// Initialization of source buffers and DMA descriptor for the DMA transfer
// (ref. CC111xFx/CC251xFx Errata Note)
static uint8_t __xdata PM3_BUF[7] = {0x07,0x07,0x07,0x07,0x07,0x07,0x04};
static uint8_t __xdata dmaDesc[8] = {0x00,0x00,0xDF,0xBE,0x00,0x07,0x20,0x42};


/***********************************************************************************
* LOCAL FUNCTIONS
*/

/***********************************************************************************
* @fn          setup_port_interrupt
*
* @brief       Function which sets up the Port 0 Interrupt for Power Mode 3 usage.
*
* @param       void
*
* @return      void
*/
void setup_port_interrupt(void)
{

    // Clear Port 0 Interrupt flags
    P0IF = 0;
    P0IFG = 0x00;

    // Enable interrupt on P0_1/2/3
    PICTL |= PICTL_P0IENL;

    // Enable CPU Interrupt for Port 0 (IEN1.P0IE = 1)
    P0IE = 1;

    // Enable Global Interrupt by setting the (IEN0.EA = 1)
    EA = 1;

}


/***********************************************************************************
* @fn          port0_isr
*
* @brief       Port 0 Interrupt Service Routine, which executes when SRF04EB S1
*              (P0_1) is pressed. Note that the [SLEEP.MODE] bits must be cleared
*              inside this ISR in order to prevent unintentional Power Mode 3 entry.
*
* @param       void
*
* @return      void
*/

void port0_isr(void) __interrupt P0INT_VECTOR
{

    // Note that the order in which the following flags are cleared is important.

    // Clear Port Module Interrupt Flag (P0IFG.P0IF1 = 0)
    P0IFG &= ~BIT1;

    // Clear CPU Interrupt Flag for P0 (IRCON.P0IF = 0)
    P0IF = 0;

    // Clear the [SLEEP.MODE] bits, because an interrupt can also occur
    // before the SoC has actually entered Power Mode 3.
    SLEEP &= ~SLEEP_MODE;

    // Set SRF04EB LED1 to indicate Power Mode 3 exit
    P1_3 = 0;

}



/***********************************************************************************
* @fn          main
*
* @brief       Enter Power Mode 3 based on CC111xFx/C251xFx Errata Note,
*              exit Power Mode 3 using Port 0 Interrupt.
*
* @param       none
*
* @return      0
*/

void main(void)
{
    volatile uint8_t storedDescHigh = DMA0CFGH;
    volatile uint8_t storedDescLow = DMA0CFGL;
    volatile uint8_t temp;

    // Initialize P1_1/3 for SRF04EB LED1/3
    P1SEL &= ~(BIT3 | BIT0);
    P1_0 = 1; P1_3 = 1;
    P1DIR |= (BIT3 | BIT0);

    // Initialize P0_1 for SRF04EB S1 button
    P0SEL &= ~BIT1;
    P0DIR &= ~BIT1;
    P0INP |= BIT1;



    // Setup and enable Port 0 Interrupt, which shall wake-up the
    // SoC from Power Mode 3.
    setup_port_interrupt();


    // Infinite loop:
    // Enter/exit Power Mode 3.
    while(1)
    {
        // Switch system clock source to HS RCOSC and max CPU speed:
        // Note that this is critical for Power Mode 3. After reset or
        // exiting Power Mode 3 the system clock source is HS RCOSC,
        // but to emphasize the requirement we choose to be explicit here.
        SLEEP &= ~SLEEP_OSC_PD;
        while( !(SLEEP & SLEEP_HFRC_S) );
        CLKCON = (CLKCON & ~CLKCON_CLKSPD) | CLKCON_OSC | CLKSPD_DIV_2;
        while ( !(CLKCON & CLKCON_OSC) ) ;
        SLEEP |= SLEEP_OSC_PD;

        // Wait some time in Active Mode, and set SRF04EB LED1 before
        // entering Power Mode 3
        for(activeModeCnt = 0; activeModeCnt < ACT_MODE_TIME; activeModeCnt++);
        P0_1 = 1;



        ///////////////////////////////////////////////////////////////////////
        ////////// CC111xFx/CC251xFx Errata Note Code section Begin ///////////
        ///////////////////////////////////////////////////////////////////////


        // Store current DMA channel 0 descriptor and abort any ongoing transfers,
        // if the channel is in use.
        storedDescHigh = DMA0CFGH;
        storedDescLow = DMA0CFGL;
        DMAARM |= (DMAARM_ABORT | DMAARM0);

        // Update descriptor with correct source.
        dmaDesc[0] = (uint16_t)&PM3_BUF >> 8;
        dmaDesc[1] = (uint16_t)&PM3_BUF;
        // Associate the descriptor with DMA channel 0 and arm the DMA channel
        DMA0CFGH = (uint16_t)&dmaDesc >> 8;
        DMA0CFGL = (uint16_t)&dmaDesc;
        DMAARM = DMAARM0;

        // NOTE! At this point, make sure all interrupts that will not be used to
        // wake from PM are disabled as described in the "Power Management Control"
        // chapter of the data sheet.

        // The following code is timing critical and should be done in the
        // order as shown here with no intervening code.

        // Align with positive 32 kHz clock edge as described in the
        // "Sleep Timer and Power Modes" chapter of the data sheet.
        temp = WORTIME0;
        while(temp == WORTIME0);

        // Make sure XOSC is powered down when entering PM{2 - 3} and that the
        // flash cache is disabled.
        MEMCTR |= MEMCTR_CACHD;
        SLEEP = 0x07;

        // Enter power mode as described in chapter "Power Management Control"
        // in the data sheet. Make sure DMA channel 0 is triggered just before
        // setting [PCON.IDLE].
        __asm nop __endasm;
        __asm nop __endasm;
        __asm nop __endasm;
        
        if(SLEEP & 0x03)
        {
        __asm MOV 0xD7,#0x01 __endasm; // DMAREQ = 0x01;
        __asm NOP __endasm; // Needed to perfectly align the DMA transfer.
        __asm ORL 0x87,#0x01 __endasm; // PCON |= 0x01;
        __asm NOP __endasm;

        }
        // End of timing critical code


        // Enable Flash Cache.
        MEMCTR &= ~MEMCTR_CACHD;

        // Update DMA channel 0 with original descriptor and arm channel if
        // it was in use before PM was entered.
        DMA0CFGH = storedDescHigh;
        DMA0CFGL = storedDescLow;
        DMAARM = DMAARM0;

        ///////////////////////////////////////////////////////////////////////
        /////////// CC111xFx/CC251xFx Errata Note Code section End ////////////
        ///////////////////////////////////////////////////////////////////////

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
