
/***********************************************************************************
  Filename:     powermode2.c

  Description:  This software example shows how to properly enter Power Mode 2
                based on the CC111xFx/CC251xFx Errata Note, and then exit upon
                Sleep Timer Interrupt. The SRF04EB LED1 is cleared before
                entering Power Mode 2, and then set by the Sleep Timer ISR.
                Hence, the SRF04EB LED1 is ON in Active Mode and OFF in
                Power Mode 2.

                Note that the system clock source must be HS RCSOC before
                entering Power Mode 2!
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

// Wait time in Active mode
#define ACT_MODE_TIME  10000


/***********************************************************************************
* LOCAL VARIABLES
*/

// Variable for active mode duration
static uint32 __xdata activeModeCnt = 0;

// Initialization of source buffers and DMA descriptor for the DMA transfer
// (ref. CC111xFx/CC251xFx Errata Note)
static uint8 __xdata PM2_BUF[7] = {0x06,0x06,0x06,0x06,0x06,0x06,0x04};
static uint8 __xdata dmaDesc[8] = {0x00,0x00,0xDF,0xBE,0x00,0x07,0x20,0x42};

static int8 EVENT0_HIGH = 0xFF;
static int8 EVENT0_LOW = 0xFF;


/***********************************************************************************
* LOCAL FUNCTIONS
*/

/***********************************************************************************
* @fn          setup_sleep_interrupt
*
* @brief       Function which sets up the Sleep Timer Interrupt
*              for Power Mode 2 usage.
*
* @param       void
*
* @return      void
*/
void setup_sleep_interrupt(void)
{
    // Clear Sleep Timer CPU Interrupt flag (IRCON.STIF = 0)
    STIF = 0;

    // Clear Sleep Timer Module Interrupt Flag (WORIRQ.EVENT0_FLAG = 0)
    WORIRQ &= ~WORIRQ_EVENT0_FLAG;

    // Enable Sleep Timer Module Interrupt (WORIRQ.EVENT0_MASK = 1)
    WORIRQ |= WORIRQ_EVENT0_MASK;

    // Enable Sleep Timer CPU Interrupt (IEN0.STIE = 1)
    STIE = 1;

    // Enable Global Interrupt (IEN0.EA = 1)
    EA = 1;
}


/***********************************************************************************
* @fn          sleep_timer_isr
*
* @brief       Sleep Timer Interrupt Service Routine, which executes when
*              the Sleep Timer expires. Note that the [SLEEP.MODE] bits must
*              be cleared inside this ISR in order to prevent unintentional
*              Power Mode 2 entry.
*
* @param       void
*
* @return      void
*/
#pragma vector = ST_VECTOR
__interrupt void sleep_timer_isr(void)
{
    // Clear Sleep Timer CPU interrupt flag (IRCON.STIF = 0)
    STIF = 0;

    // Clear Sleep Timer Module Interrupt Flag (WORIRQ.EVENT0_FLAG = 0)
    WORIRQ &= ~WORIRQ_EVENT0_FLAG;

    // Clear the [SLEEP.MODE] bits, because an interrupt can also occur
    // before the SoC has actually entered Power Mode 2.
    SLEEP &= ~SLEEP_MODE;

    // Set SRF04EB LED1 to indicate Power Mode 2 exit
    P1_0 = 0;
}




/***********************************************************************************
* @fn          main
*
* @brief       Enter Power Mode 2 based on CC111xFx/CC251xFx Errata Note,
*              exit Power Mode 2 using Sleep Timer Interrupt.
*
* @param       none
*
* @return      0
*/

void main(void)
{
    volatile uint8 storedDescHigh = DMA0CFGH;
    volatile uint8 storedDescLow = DMA0CFGL;
    volatile int8 temp;

    // Initialize P1_1/3 for SRF04EB LED1/3
    P1SEL &= ~(BIT3 | BIT0);
    P1_0 = 1; P1_3 = 1;
    P1DIR |= (BIT3 | BIT0);

    // Setup + enable the Sleep Timer Interrupt, which is
    // intended to wake-up the SoC from Power Mode 2.
    setup_sleep_interrupt();



    // Infinite loop:
    // Enter/exit Power Mode 2.
    while(1)
    {
        // Switch system clock source to HS RCOSC and max CPU speed:
        // Note that this is critical for Power Mode 2. After reset or
        // exiting Power Mode 2 the system clock source is HS RCOSC,
        // but to emphasize the requirement we choose to be explicit here.
        SLEEP &= ~SLEEP_OSC_PD;
        while( !(SLEEP & SLEEP_HFRC_S) );
        CLKCON = (CLKCON & ~CLKCON_CLKSPD) | CLKCON_OSC | CLKSPD_DIV_2;
        while ( !(CLKCON & CLKCON_OSC) ) ;
        SLEEP |= SLEEP_OSC_PD;

        // Set LS XOSC as the Sleep Timer clock source (CLKCON.OSC32 = 0)
        CLKCON &= ~CLKCON_OSC32;

        // Wait some time in Active Mode, and set SRF04EB LED1 before
        // entering Power Mode 2
        for(activeModeCnt = 0; activeModeCnt < ACT_MODE_TIME; activeModeCnt++);
        P1_0 = 1;



        ///////////////////////////////////////////////////////////////////////
        ////////// CC111xFx/CC251xFx Errata Note Code section Begin ///////////
        ///////////////////////////////////////////////////////////////////////

        // Store current DMA channel 0 descriptor and abort any ongoing transfers,
        // if the channel is in use.
        storedDescHigh = DMA0CFGH;
        storedDescLow = DMA0CFGL;
        DMAARM |= (DMAARM_ABORT | DMAARM0);

        // Update descriptor with correct source.
        dmaDesc[0] = (uint16)&PM2_BUF >> 8;
        dmaDesc[1] = (uint16)&PM2_BUF;
        // Associate the descriptor with DMA channel 0 and arm the DMA channel
        DMA0CFGH = (uint16)&dmaDesc >> 8;
        DMA0CFGL = (uint16)&dmaDesc;
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

        // Set Sleep Timer Interval
        WOREVT0 = EVENT0_HIGH;
        WOREVT1 = EVENT0_LOW;

        // Make sure HS XOSC is powered down when entering PM{2 - 3} and that
        // the flash cache is disabled.
        MEMCTR |= MEMCTR_CACHD;
        SLEEP = 0x06;

        // Enter power mode as described in chapter "Power Management Control"
        // in the data sheet. Make sure DMA channel 0 is triggered just before
        // setting [PCON.IDLE].
        asm("NOP");
        asm("NOP");
        asm("NOP");
        if(SLEEP & 0x03)
        {
            asm("MOV 0xD7,#0x01");      // DMAREQ = 0x01;
            asm("NOP");                 // Needed to perfectly align the DMA transfer.
            asm("ORL 0x87,#0x01");      // PCON |= 0x01 -- Now in PM2;
            asm("NOP");                 // First call when awake
        }
        // End of timing critical code

        // Enable Flash Cache.
        MEMCTR &= ~MEMCTR_CACHD;

        // Update DMA channel 0 with original descriptor and arm channel if it was
        // in use before PM was entered.
        DMA0CFGH = storedDescHigh;
        DMA0CFGL = storedDescLow;
        DMAARM = DMAARM0;

        ///////////////////////////////////////////////////////////////////////
        /////////// CC111xFx/CC251xFx Errata Note Code section End ////////////
        ///////////////////////////////////////////////////////////////////////



        // Wait until HS RCOSC is stable
        while( !(SLEEP & SLEEP_HFRC_S) );

        // Set LS XOSC as the clock oscillator for the Sleep Timer (CLKCON.OSC32 = 0)
        CLKCON &= ~CLKCON_OSC32;
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
