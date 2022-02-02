/***********************************************************************************
  Filename:     t1_free.c

  Description:  Uses Timer 1 to function in Free Running mode. Channel 0 is used to
                toggle P0_2 when counter reaches compare value (channel 0).
                Interrupt is generated when Timer 1 counter reaches compare value.
                The green LED is toggled when the channel 0 interrupt occurs.
                The LED should blink about 3 times per second because of the
                frequency used. The variable timer1Ch0CmpResult can be used for
                debugging.


                Settings:
                    - Channel 0
                    - Free-running mode
                    - Output compare mode with toggling on compare
                    - Interrupt generated on compare
                    - Default clock source and speed (HS RCOSC at 13 Mhz)
                    - 402.25 kHz tickspeed


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

// Variable defined for debugging/testing purpose.
static uint16 timer1Ch0CmpResult;

/***********************************************************************************
* LOCAL FUNCTIONS
*/

/***********************************************************************************
* @fn          timer1_ISR
*
* @brief       ISR for Timer 1. Interrupt occurs when a compare event occurs on
*              channel 0. Toggles green LED. CPU interrupt flag (IRCON) for
*              Timer 1 is cleared automatically by hardware
*
* @param       void
*
* @return      void
*/

#pragma vector = T1_VECTOR
__interrupt void timer1_ISR(void)
{
    // Clear Timer 1 channel 0 interrupt flag. The clearing has to be like this
    // to avoid loosing other Timer 1 interrupts (writing 1 to T1CTL[7:4] has
    // no effect as these are R/W0 accessible)
    T1CTL = (~T1CTL_CH0IF & 0xF0) | (T1CTL & 0x0F);


    // This value should be the same as the value set for T1CC0 (T1CC0L and T1CC0H)
    // in the setup. When reading this value while debugging, some delay should
    // be expected due to interrupt generation time and other time consuming code.
    // Because of this, a value higher than T1CC0 will be read.

    timer1Ch0CmpResult = T1CNTL;
    timer1Ch0CmpResult |= (T1CNTH << 8);

    P1_0 ^= 1;  // Toggle SRF04EB LED1
}

/***********************************************************************************
* @fn          main
*
* @brief       Uses Timer 1 to function in Free Running mode. Channel 0 is used to
*              toggle P0_2 when counter reaches compare value (channel 0).
*              Interrupt is generated when Timer 1 counter reaches compare value.
*              Green LED is toggled when channel 0 interrupt occurs.
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

    // Clear Timer 1 channel 0 interrupt flag
    // CPU interrupt flag (IRCON) for Timer 1 is cleared automatically by hardware.
    T1CTL &= ~T1CTL_CH0IF;

    // Set individual interrupt enable bit in the peripherals SFR
    T1CCTL0 |= T1CCTL0_IM;      // Enable interrupt on channel 0
    T1CCTL1 &= ~T1CCTL1_IM;     // Disable interrupt on channel 1
    T1CCTL2 &= ~T1CCTL2_IM;     // Disable interrupt on channel 2
    OVFIM = 0;                  // Disable overflow interrupt

    // Enable Timer 1 interrupts by setting [IEN1.T1IE=1]
    T1IE = 1;

    // Enable global interrupt by setting the [IEN0.EA=1]
    EA = 1;

    /***************************************************************************
     * Setup peripheral I/O for Timer
     *
     * We can also choose the Alternative 2 location for Timer 1, or for the peripherals
     * that use the same pins as Timer 1. This can be done by setting PERCFG-register
     */

    // Select P0_2 for peripheral function
    P0SEL |=  BIT2 ;

    // Set P2DIR to prioritize Timer 1's control over port 0 pins for
    // channel 0 over USART 0
    P2DIR = (P2DIR & ~P2DIR_PRIP0) | P2DIR_PRIP0_2;

    /***************************************************************************
     * Setup Timer settings
     *
     * Here we will select which channel(s) that will be used. We can choose to
     * use them in compare mode or capture mode.
     *
     * We can also select what mode the Timer shall operate on. When the mode is
     * selected, the Timer will start to run. Please see the data sheet for more
     * information.
     *
     * Notes:
     * - T1CCO is used by channel 1 & 2 for some compare modes, in case channels
     *   are used simultaneously.
     * - In compare mode using modulo mode or up-down mode, channel 0 will
     *   generate spike signals when [T1CCTL0.CMP = 3 or 4] since T1CC0 will
     *   then be both the compare value and the overflow value.
     * - The input signal (pulse), when in capture mode, must have a duration
     *   longer than the system clock.
     * - To choose 26 Mhz clock speed, HS XOSC must be chosen as system clock
     *   source.
     */

    // Set channel 0 to compare mode and to toggle on compare.
    T1CCTL0 = (T1CCTL0 & ~T1CCTL0_CMP) | T1C0_TOG_ON_CMP | T1CCTL0_MODE;

    // Set compare register of channel 0 to 32767 ( 0xFFFF / 2 )
    T1CC0L = 0xFF;
    T1CC0H = 0x7F;

    // Set prescaler divider value to 32 to get a tickspeed of 402,25 kHz and
    // set Timer 1 to free running mode
    T1CTL = (T1CTL & ~(T1CTL_MODE | T1CTL_DIV)) | T1CTL_MODE_FREERUN
        | T1CTL_DIV_32;

    // Timer 1 will now start counting...

    while(1);


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
