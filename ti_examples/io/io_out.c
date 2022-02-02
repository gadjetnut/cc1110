
/***********************************************************************************
  Filename:     io_out.c

  Description:  Setup P0_0 to generate interrupts when output is changed.
                Button S1 on SMARTRF04EB is used to toggle P0_0.

                When a rising edge is detected on the pin, the ISR for the pin
                will be executed. This will toggle the green LED.

                NOTE:
                Some pins on SMARTRF04EB are not mapped to the corresponding
                pins on cc2510. The pins and their mappings are:

                      CC2510    |   SRF04EB
                      *********************
                       P1_2     |   P2_3
                       P2_0     |   P2_4
                       P2_3     |   none

                The debug interface uses the I/O pins P2_1 as Debug Data and
                P2_2 as Debug Clock during Debug mode. These I/O pins can be
                used as general purpose I/O only while the device is not in
                Debug mode. Thus the debug interface does not interfere with any
                peripheral I/O pins. If interrupt is enabled for port 2, the
                interrupt will be triggered continuously when in Debug mode.

                Furthermore, note that for some SRF04EB versions P0_0 is connected
                to the Audio/MIC interface through the R9 resistor. In order to avoid
                unintentional signal toggling on P0_0 please remove this resistor.

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


/*******************************************************************************
* @fn          p0_ISR
*
* @brief       Interrupt Service Routine for port 0. Clears flags for pin 0
*              and pin 1, and toggles green LED. Pin 1 is cleared because
*              button S1 on SMARTRF04EB is used
*
* @param       void
*
* @return      void
*/

#pragma vector = P0INT_VECTOR
__interrupt void p0_ISR(void)
{

    /* Note that the order in which the following flags are cleared is important.
       For level triggered interrupts (port interrupts) one has to clear the module
       interrupt flag prior to clearing the CPU interrupt flags. */

    if (P0IFG & BIT0)
    {
        P1_0 ^= 1;           // Toggle SRF04EB LED1
        P0IFG &= ~BIT0;      // Clear status flag for pin
    }

    /* Since button S1 is connected to P0_1, it will generate a port 0
       interrupt. This happens because interrupts from pins 0-3 have to be
       enabled simultaneously. This is, however, not case with port 1 where
       it is possible to enable interrupts from pins individually. */

    if (P0IFG & BIT1)
    {
        P0IFG &= ~BIT1;      // Clear status flag for pin
    }

    P0IF = 0;               // Clear CPU interrupt status flag for P0
}


/***********************************************************************************
* @fn          main
*
* @brief       Setup one pin on port 0 to generate interrupts when output is
*              changed
*
* @param       void
*
* @return      0
*/
int main(void)
{
    /***************************************************************************
     * Setup test stuff
     *
     */

    // Initialize P1_1/3 for SRF04EB LED1/3
    P1SEL &= ~(BIT3 | BIT0);
    P1_0 = 1; P1_3 = 1;
    P1DIR |= (BIT3 | BIT0);

    // Initialize P0_1 for SRF04EB S1 button
    P0SEL &= ~BIT1;
    P0DIR &= ~BIT1;
    P0INP |= BIT1;


    /***************************************************************************
     * Setup interrupt
     *
     */

    // Clear interrupt flags for P0
    P0IFG &= ~(BIT0 | BIT1 | BIT2 | BIT3);  // Clear status flag for pins 0-3
    P0IF = 0;                               // Clear CPU interrupt status flag for P0

    // Set individual interrupt enable bit in the peripherals SFR.
    // Enable interrupt from pins 0-3 on P0
    // Set interrupt on rising edge and for P0
    PICTL = (PICTL & ~PICTL_P0ICON) | PICTL_P0IENL;

    // Enable P0 interrupts
    P0IE = 1;

    // Enable global interrupt by setting the [IEN0.EA=1]
    EA = 1;

    /***************************************************************************
     * Setup I/O
     *
     */

    // Select function
    P0SEL &= ~BIT0;             // Select to function as General Purpose I/O

    // Select direction
    P0DIR |= BIT0;              // Select direction to output

    /* Now everything is setup */
    while(1)
    {
        // if button is pushed, toggle P0_0
        if ( !(P0 & BIT1) )
        {
            P0_0 ^= 1;
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
