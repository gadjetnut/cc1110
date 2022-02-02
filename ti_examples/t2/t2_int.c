
/***********************************************************************************
  Filename:     t2_int.c

  Description:  This example uses Timer 2 to generate interrupt after a predefined
                period. The green LED is toggled when interrupts occur.

                Settings:
                    - Free-running mode
                    - Interrupt generated when counter reaches 0x00
                    - 13 Mhz clock tickspeed
                    - 3.125 Hz counter speed



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


/***********************************************************************************
* LOCAL VARIABLES
*/

/***********************************************************************************
* LOCAL FUNCTIONS
*/

// Interrupt routine for Timer 2
#pragma vector = T2_VECTOR
__interrupt void timer2_interrupt(void)
{
    // We don't need to clear any flags here since Timer 2 doesn't have any
    // peripheral flags, and the CPU flag (IRCON.T2IF) is cleared automatically
    // by hardware

    P1_0 ^= 1;  // Toggle SRF04EB LED1
}



/***********************************************************************************
* @fn          main
*
* @brief       This example uses Timer 2 to generate interrupt after a predefined
*              period. The green LED is toggled when interrupts occur.
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
    P1_0 = 1; P1_3 = 1;
    P1DIR |= (BIT3 | BIT0);

    /***************************************************************************
    * Setup interrupt
    */

    // Clear Timer 2 interrupt flags
    // CPU interrupt flag (IRCON.T2IF) is cleared automatically by hardware

    // Set individual interrupt enable bit in the peripherals SFR
    T2CTL |= T2CTL_INT;             // Enable interrupt by setting [T2CTL.INT=1]

    // Enable Timer 2 interrupts by setting [IEN1.T1IE]
    T2IE = 1;

    // Enable global interrupt by setting the [IEN0.EA=1]
    EA = 1;

    /***************************************************************************
     * Setup Timer settings
     *
     * By setting [T2CTL.TIG = 1], the Timer will be counting down in
     * free-running mode and wrap around when 0x00 is reached. By setting
     * [T2CTL.TIG = 0], the Timer will be counting down starting from the current
     * T2CT value and stop when 0x00 is reached. Obviously, if this value is
     * 0x00 at the beginning, the timer will do nothing.
     */

    // Set timer 2 interval / timeslot and start Timer 2

    // Set Timer 2 prescalar multiplier value (0x00 is interpreted as 256)
    T2PR = 0xFF;

    // Set [T2CTL.TIP] to 64 and start Timer 2 in free running mode
    T2CTL = (T2CTL & ~T2CTL_TIP) | T2CTL_TIP_64 | T2CTL_TIG;

    /* This gives the counter speed for T2CT of (256 * 64) / 13 * 10^6 = 1.26 ms
       which then gives a period of 1.26 * 10^-3 * 256 = 0.32 s (3.125 Hz),
       since counter runs up to 256 */

    while (1);

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
