/*******************************************************************************
  Filename:    t3_down.c

  Description: Runs Timer 3 in down mode. Initially, the green LED is turned
               on. When the Timer has counted down to 0x00, an interrupt occurs,
               it turns the green LED off and the yellow LED on. The LEDs are
               in no way directly related to the Timer, they are just used to
               indicate what is happening.

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

#include <hal_types.h>
#include <hal_defs.h>
#include <hal_cc8051.h>
#include <ioCCxx10_bitdef.h>


/*******************************************************************************
* CONSTANTS
*/


/*******************************************************************************
* LOCAL VARIABLES
*/

/* Variable that terminates the program when set to zero by t3_isr(). */
static uint8 running;


/*******************************************************************************
* LOCAL FUNCTIONS
*/

/*******************************************************************************
* @fn      t3_isr
*
* @brief   Interrupt handler for Timer 3 overflow interrupts. The green LED is
*          turned off and the yellow LED is turned on. Interrupts from Timer 3
*          are level triggered, so the module interrupt flag is cleared
*          prior to the CPU interrupt flag.
*
* @param   void
*
* @return  void
*
*******************************************************************************/
#pragma vector = T3_VECTOR
__interrupt void t3_isr(void)
{
    /* Clears the module interrupt flag. */
    T3OVFIF = 0;

    /* Turns off the green LED, turns on the yellow LED. */
    P1_0 = 1;   // Clear SRF04EB LED1
    P1_3 = 0;   // Set SRF04EB LED3

    /* Makes the program terminate by setting the running variable to 0. */
    running = 0;

    /* Clears the CPU interrupt flag. */
    T3IF = 0;
}

/*******************************************************************************
* @fn          main
*
* @brief       LED1, LED3 and Timer 3 are all initialized. LED1 is turned on.
*              The rest of the program is executed by an interrupt from Timer 3.
*              This interrupt occurs when the Timer has counted down from T3CC0
*              to 0x00, and it turns off LED1 and turns on LED3. The LEDs are in
*              no way directly related to the Timer, they are just used to
*              indicate what is happening.
*
* @param       void
*
* @return      0
*******************************************************************************/
int main(void)
{
    /* Initialize P1_1/3 for SRF04EB LED1/3 */
    P1SEL &= ~(BIT3 | BIT0);
    P1_0 = 1; P1_3 = 1;
    P1DIR |= (BIT3 | BIT0);


    /* Clock control. Configures the Timer tick speed setting, division by 128.
     * This results in a Timer tick frequency of 203.125 kHz.
     */
    CLKCON = (CLKCON & ~CLKCON_TICKSPD) | (7 << 3);


    /* Timer 3 channel 0 compare value. Sets the initial value which the Timer
     * is to count down from.
     */
    T3CC0 = 0xF0;

    /* Timer 3 control. Configuration:
     * - Prescaler divider value: 128.
     * - Interrupts enabled.
     * - Down mode.
     * The Timer is also cleared and started.
     */
    T3CTL = T3CTL_DIV_128 | T3CTL_START | T3CTL_OVFIM | T3CTL_MODE_DOWN;

    /* Enables global interrupts (IEN0.EA = 1) and interrupts from Timer 3
     * (IEN1.T3IE = 1).
     */
    EA = 1;
    T3IE = 1;

    /* Turns on the green LED. */
    P1_0 = 0;   // Set SRF04EB LED1

    /* Loop until the running variable is set to 0 by t3_isr(). */
    running = 1;
    while (running);

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
