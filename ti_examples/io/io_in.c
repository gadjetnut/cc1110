
/***********************************************************************************
  Filename:     io_in.c

  Description:  Setup one pin on port 1 to generate interrupts on incoming events.

                When a rising edge is detected on the pin, the ISR for the pin
                will be executed. This will toggle the red LED on SRF04EB.

                The pin can be chosen using the PIN macro.

                To test this, the pin that is set up, can be toggled with GND,
                for example using a wire from GND to the pin.

                NOTE: Some pins on SMARTRF04EB are not mapped to the
                      corresponding pins on cc2510. The pins and their mappings
                      are

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

/* Choose which pin to configure. BIT1 will configure pin 1, BIT2 will configure
   pin 2 and so on */

#define PIN BIT5


/***********************************************************************************
* LOCAL VARIABLES
*/


/***********************************************************************************
* LOCAL FUNCTIONS
*/


/*******************************************************************************
* @fn          p1_ISR
*
* @brief       Interrupt Service Routine for port 1 and the chosen pin.
               Clears interrupt flags and toggles LED.
*
* @param       void
*
* @return      void
*/

#pragma vector = P1INT_VECTOR
__interrupt void p1_ISR(void)
{
    /* Note that the order in which the following flags are cleared is important.
       For level triggered interrupts (port interrupts) one has to clear the module
       interrupt flag prior to clearing the CPU interrupt flags. */

    P1IFG &= ~PIN;              // Clear status flag for pin
    P1IF = 0;                   // Clear CPU interrupt status flag for P1

    P1_3 ^= 1;                  // Toggle SRF04EB LED3

}



/***********************************************************************************
* @fn          main
*
* @brief       Setup one pin on port 1 to generate interrupts on incoming events.
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


    /***************************************************************************
     * Setup interrupt
     *
     */

    // Clear interrupt flags for P1
    P1IFG &= ~PIN;              // Clear status flag for pin
    P1IF = 0;                   // Clear CPU interrupt status flag for P1

    // Set individual interrupt enable bit in the peripherals SFR.
    P1IEN |= PIN;               // Enable interrupt from pin
    PICTL |= PICTL_P1ICON;      // Set interrupt on rising edge and for P1

    // Enable P1 interrupts
    IEN2 |= IEN2_P1IE;

    // Enable global interrupt by setting the IEN0.EA=1
    EA = 1;

    /***************************************************************************
     * Setup I/O
     *
     */

    // Select function
    P1SEL &= ~PIN;              // Select to function as General Purpose I/O

    // Select direction
    P1DIR &= ~PIN;              // Select direction to input

    // Select input type
    P1INP &= ~PIN;              // Pull up/pull down

    /* Now everything is set up and the rest of program will run through the ISR */

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
