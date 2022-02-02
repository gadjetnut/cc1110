/*******************************************************************************
  Filename:    t4_pwm.c

  Description: Generates a sine wave signal, by using Timer 4 to generate
               a PWM signal with varying duty cycle. The frequency f of the
               sine wave is
                             f = f_tts / (t4_div * 2 * t4_tcv * N)
               where f_tts is the Timer tick speed (can be adjusted in the
               CLKCON register), t4_div is the Timer 4 prescaler divider value
               (can be adjusted in the T4CTL register), t4_tcv is the Timer 4
               terminal count value (can be adjusted in the T4CC0 register) and
               N is the number of samples per sine wave. This example uses
               f_tts = 13 kHz (default), t4_div = 4, t4_tcv = 203 and N = 16
               to create a 500 Hz sine wave.

               The values for adjusting the PWM duty cycle are stored in a
               lookup table, and this value sets the compare value for
               Timer 4, which is running in up/down mode, setting output on
               compare-up and clearing output on compare-down. The compare
               value is changed when Timer 4 reaches 0x00 (and generates an
               interrupt).

               On the SmartRF04 EB, Timer 4 is connected P1_1 when I/O location
               alternative 1 is chosen. P1_1 functions as PWM audio output,
               see the CC2510 Development Kit User manual for details on the
               audio interface. To function as audio output, P1_1 must be a
               peripheral I/O pin. Not all revisions of the SmartRF04 EB has
               the R104 resistor connected. This may give an unexpected audio
               output, see the SmartRF04 EB documentation for details.

*******************************************************************************/

/*******************************************************************************
* INCLUDES
*/
#include "cc1110.h"

/*******************************************************************************
* CONSTANTS
*/

/* Number of sine samples. */
#define NO_SAMPLES 16

/*******************************************************************************
* LOCAL VARIABLES
*/

/*******************************************************************************
* LOCAL FUNCTIONS
*/

/*******************************************************************************
* @fn      t4_isr
*
* @brief   Interrupt handler for Timer 4 overflow interrupts. Interrupts from
*          Timer 4 are level triggered, so the module interrupt flag is cleared
*          prior to the CPU interrupt flag. Timer 4 interrupt source is not
*          checked, since only overflow interrupts are enabled. On Timer 4
*          overflow interrupts, the PWM duty cycle is changed by altering the
*          compare value.
*
* @param   void
*
* @return  void
*
*******************************************************************************/
#pragma vector = T4_VECTOR
__interrupt void t4_isr(void)
{
    /* Local variables. The sine wave consists of 16 samples, so the PWM
     * lookup table pwmLut has 16 values. These are calculated from
     * 81 * cos( k * 360/16 ) + T4CC0 / 2, k = 0, 1, ..., 15. Since a too large
     * amplitude may cause timing problems related to the compare register not
     * being changed fast enough, the amplitude must not bee to large. 81 is
     * found to work.
     */
    static uint8 i = 0;
    static const uint16 pwmLut[NO_SAMPLES] = {183, 176, 159, 132, 102, 71, 44,
                                        27, 21, 27, 44, 71, 102, 132, 159, 176};

    /* Clears the module interrupt flag. */
    T4OVFIF = 0;

    /* Writes the PWM value to the compare register and increments the
     * index. Writing to the compare register TxCC1 takes effect immediately.
     */
    T4CC1 = pwmLut[i++];
    if (i == NO_SAMPLES)
    {
        i = 0;
    }

    /* Clears the CPU interrupt flag. */
    T4IF = 0;
}

/*******************************************************************************
* @fn          main
*
* @brief       Configures P1_1 as Timer 4 output, sets up Timer 4 for centre-
*              aligned PWM operation and enables overflow interrupts. The rest
*              of the program is interrupt driven, so an infinite loop is needed
*              after the initialization.
*
* @param       void
*
* @return      0
*******************************************************************************/
int main(void)
{
    /* Timer 4 output configuration. Selects alternative 1 I/O location for
     * Timer 4 in order to use P1_1 as output. Configures P1_1 as a peripheral
     * I/O pin.
     */
    PERCFG &= ~PERCFG_T4CFG;
    P1SEL |= BIT1;

    /* Timer 4 channel 1 compare control configuration. Selects the output mode
     * so that the output is set on compare-up and cleared on compare-down and
     * enables compare mode. This also disables compare interrupts.
     */
    T4CCTL1 = T4CCTL1_SET_CMP_UP_CLR_0 | T4CCTL1_MODE;

    /* Timer 4 channel 0 compare value. Sets the Timer 4 terminal count value
     * to 0xCF, which makes the Timer count up to 0xCF before counting down to
     * 0x00. Hence it controls the PWM period.
     */
    T4CC0 = 0xCB;

    /* Timer 4 channel 1 compare value. Sets the initial compare value to 0xB7
     * (183). This value controls the pulse width and is changed at each Timer
     * overflow interrupt.
     */
    T4CC1 = 0xB7;

    /* Enables global interrupts and interrupts from Timer 4. */
    EA = 1;
    T4IE = 1;

    /* Timer 4 control. Sets the prescaler divider value to 4, starts the Timer,
     * enables overflow interrupts, clears the Timer and sets the mode to
     * up-down.
     */
    T4CTL = T4CTL_DIV_4 | T4CTL_START | T4CTL_OVFIM |
        T4CTL_CLR | T4CTL_MODE_UPDOWN;

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
