/*******************************************************************************
  Filename:    t3_modulo.c

  Description: Runs Timer 3 in modulo mode. The terminal count value register
               T3CC0 (overflow) is altered at each overflow interrupt. The green
               LED is also toggled at each interrupt. The LED is in no way
               directly related to the Timer, it is just used to indicate
               Timer overflow.

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


/*******************************************************************************
* LOCAL FUNCTIONS
*/

/*******************************************************************************
* @fn      t3_isr
*
* @brief   Interrupt handler for Timer 3 overflow interrupts. Interrupts from
*          Timer 3 are level triggered, so the module interrupt flag is cleared
*          prior to the CPU interrupt flag. The green LED is toggled and the
*          overflow interval (and hence the toggle interval) is adjusted.
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

    /* Toggles the green LED. */
    P1_0 ^= 1;

    /* Changes the Timer overflow value to change the LED blinking interval.
     * The value increases and decreases linearly between 0xFF and 0x02.
     * Writing to the compare register T3CC0 does not take effect on the
     * output compare value until the counter value  is 0x00. (Writing to
     * the compare register T3CC1 takes effect immediately.)
     */
    static int8 sign = -1;
    if (T3CC0 == 0xFF || T3CC0 == 0x02)
    {
        sign *= -1;
    }
    T3CC0 += sign;

    /* Clears the CPU interrupt flag. */
    T3IF = 0;
}

/*******************************************************************************
* @fn          main
*
* @brief       LED1 and Timer 3 are both initialized. The rest of the program
*              is executed by interrupts from Timer 3. The LED is in no way
*              related to the Timer, it is just used to indicate Timer overflow.
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

    /* Clock control. Configures the Timer tick speed setting, resulting in a
     * Timer tick frequency of 203.125 kHz.
     */
    CLKCON = (CLKCON & ~CLKCON_TICKSPD) | TICKSPD_DIV_128;

    /* Timer 3 channel 0 compare value. Sets the Timer overflow value to 0x02,
     * i.e., overflow after 2 ticks. This value is changed at each Timer
     * overflow interrupt.
     */
    T3CC0 = 0x02;

    /* Timer 3 control. Configuration:
     * - Prescaler divider value: 128.
     * - Interrupts enabled.
     * - Modulo mode.
     * The Timer is also cleared and started.
     */
    T3CTL = T3CTL_DIV_128 | T3CTL_START | T3CTL_OVFIM |
        T3CTL_CLR | T3CTL_MODE_MODULO;

    /* Enables global interrupts (IEN0.EA = 1) and interrupts from Timer 3
     * (IEN1.T3IE = 1).
     */
    EA = 1;
    T3IE = 1;

    /* Infinite loop. */
    while (1);

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
