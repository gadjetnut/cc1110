
/***********************************************************************************
  Filename:     sleep_int.c

  Description:  Sets up Sleep Timer and Sleep Timer interrupt.

  Comments:     This code example shows how to setup the Sleep Timer for using
                it as the source for waking up from a Power Mode. In the example
                the system goes into Power Mode 0 after setting up the Sleep
                Timer and Sleep Timer interrupt. The Sleep Timer is set up to
                generate interrupt every second. When awaken, the ISR toggles
                the green LED.

                The clock source for the Sleep Timer is 32.768 LS XOSC

                Note that in order to use PM {1-2}, it will be required to
                setup PM {1-2}, but the setup of Sleep Timer shown here will be
                the same.

***********************************************************************************/

/***********************************************************************************
* INCLUDES
*/
#include "../../lib/cc1110.h"
#include "../../lib/cc1110-ext.h"


/***********************************************************************************
* CONSTANTS
*/

/***********************************************************************************
* LOCAL VARIABLES
*/


/***********************************************************************************
* LOCAL FUNCTIONS
*/

/***********************************************************************************
* @fn          sleep_isr
*
* @brief       Interrupt service routine for the sleep timer which wakes the
*              system from Power Mode. When awake the flags are cleared and
*              the green LED is toggled.
*
* @param       void
*
* @return      void
*/

void sleep_isr(void) __interrupt ST_VECTOR 
{

   /* Note that the order in which the following flags are cleared is important.
      For pulse or egde triggered interrupts one has to clear the CPU interrupt
      flag prior to clearing the module interrupt flag. */

    // Toggle SRF04EB LED1
    P1_3 ^= 1;

    // Clear [IRCON.STIF] (Sleep Timer CPU interrupt flag)
    STIF = 0;

    // Clear [WORIRQ.EVENT0_FLAG] (Sleep Timer peripheral interrupt flag)
    WORIRQ &= ~WORIRQ_EVENT0_FLAG;
}


/***********************************************************************************
* @fn          main
*
* @brief       Setup Sleep Timer and Sleep Timer interrupt. Goes in and out of
*              Power Mode 0. Note that in order to use PM {1-2}, a setup will
*              be required to setup PM {1-2}, but the setup of Sleep Timer
*              shown here will be the same.
*
* @param       void
*
* @return      0
*/

int main(void)
{
    /***************************************************************************
     * Setup test stuff
     */
    // Initialize P1_1/3 for SRF04EB LED1/3
    P1SEL &= ~(BIT3 | BIT0);
    P1_0 = 1; P1_3 = 0;
    P1DIR |= (BIT3 | BIT0);



    /***************************************************************************
     * Setup interrupt
     */

    // Clear interrupt flags
    // Clear [IRCON.STIF] (Sleep Timer CPU interrupt flag)
    STIF = 0;

    // Clear [WORIRQ.EVENT0_FLAG] (Sleep Timer peripheral interrupt flag)
    WORIRQ &= ~WORIRQ_EVENT0_FLAG;

    // Set individual interrupt enable bit in the peripherals SFR
    WORIRQ |= WORIRQ_EVENT0_MASK;    // Enable interrupt mask for sleep timer

    // Set the individual, interrupt enable bit [IEN0.STIE=1]
    STIE = 1;

    // 1.4 Enable global interrupt by setting the [IEN0.EA=1]
    EA = 1;


    /***************************************************************************
     * Setup Sleep Timer
     */

    // Select 32.768 kHz LS XOSC
    // Note that this must be done while the system clock source is HS RCOSC!
	while(CLKCON & 0x40);
    CLKCON &= ~CLKCON_OSC32;

    /* Now the time between two consecutive Event 0’s is decided by:
       t = EVENT0 * 2^(5*WOR_RES) / 32768
       By using EVENT0 = 32 and WOR_RES = 2, t = 1 s. So by using these values,
       a Sleep Timer interrupt will be generated every second. */

    // Set [WORCTL.WOR_RES = 2]
    WORCTL = (WORCTL & ~WORCTL_WOR_RES)  | WORCTL_WOR_RES_1024;

    /* Must wait for 2 clock periods after resetting the Sleep Timer before
       setting EVENT0 value */

    // Reset timer and set EVENT0 value.
    WORCTL |= WORCTL_WOR_RESET;             // Reset Sleep Timer
    char temp = WORTIME0;
    while(temp == WORTIME0);                // Wait until a positive 32 kHz edge
    temp = WORTIME0;
    while(temp == WORTIME0);                // Wait until a positive 32 kHz edge
    WOREVT0 = 0x20;                         // Use 32 for EVENT0 value
    WOREVT1 = 0x00;

    // Select Power Mode 0 (SLEEP.MODE = 0).
    SLEEP = (SLEEP & ~SLEEP_MODE) | SLEEP_MODE_PM0;

    // Note:
    // For PM2 entry there are special DMA setup requirements
    // (ref. CC111xFx/CC251xFx Errata Note), so the below code is
    // invalid for PM2 entry.

    // Enter Power Mode
    PCON |= PCON_IDLE;

    /* The system will now wake up when Sleep Timer interrupt occurs. When awake,
       the system will start enter/exit Power Mode 0 using the loop below. */

    while(1)
    {
        // Alignment of entering PM{0 – 2} to a positive edge on the 32 kHz clock source
        temp = WORTIME0;
        while(temp == WORTIME0);    // Wait until a positive 32 kHz edge
        PCON |= PCON_IDLE;          // Go into Power Mode

    }

    return 0;

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
