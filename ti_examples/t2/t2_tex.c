/***********************************************************************************
  Filename:     t2_tex.c

  Description:  This example uses Timer 2 poll on [T2CTL.TEX] which is set when
                the Timer reaches 0x00. The green LED is toggled when [T2CTL.TEX]
                is set.

                Settings:
                    - Free-running mode
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


/***********************************************************************************
* @fn          main
*
* @brief       This example uses Timer 2 poll on [T2CTL.TEX] which is set when
*              the Timer reaches 0x00. The green LED is toggled when [T2CTL.TEX]
*              is set.
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
     * Setup timer settings
     *
     * By setting [T2CTL.TIG = 1], the timer will be counting down in
     * free-running mode and wrap around when 0x00 is reached. By setting
     * [T2CTL.TIG = 0], the timer will be counting down starting from the current
     * T2CT value and stop when 0x00 is reached. Obviously, if this value is
     * 0x00 at the beginning, the timer will do nothing.
     */

    // Set timer 2 interval / timeslot and start Timer 2

    // Set Timer 2 prescaler multiplier value (0x00 is interpreted as 256)
    T2PR = 0xFF;

    // Set [T2CTL.TIP] to 64 and start timer 2 in free running mode
    T2CTL = (T2CTL & ~T2CTL_TIP) | T2CTL_TIP_64 | T2CTL_TIG;

    /* This gives the counter speed for T2CT of (256 * 64) / 13 * 10^6 = 1.26 ms
       which then gives a period of 1.26 * 10^-3 * 256 = 0.32 s (3.125 Hz),
       since counter runs up to 256 */

    while(1)
    {
        // Wait until [T2CTL.TEX = 1]
        if (T2CTL & T2CTL_TEX)
        {
            T2CTL &= ~T2CTL_TEX;
            P1_0 ^= 1;  // Toggle SRF04EB LED1
        }
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
