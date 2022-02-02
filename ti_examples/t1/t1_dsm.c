/***********************************************************************************
  Filename:      t1_dsm.c

  Description:   Uses Timer 1 channel 1 in DSM mode to generate a sine tone
                 at 500 Hz. The sine tone can be listened to through the headphone
                 output on SMARTRF04EB.

                 Settings:
                    - Channel 1
                    - Delta-Sigma Modulation mode
                    - Interrupt generated on overflow, i.e. when T1CC0 is reached.
                      New sample is written when interrupt occurs
                    - 26 MHz tickspeed
                    - 16 sine samples as input to DSM which then generate a
                      500 Hz sine signal


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

#define SAMPLE_SIZE 16

/***********************************************************************************
* LOCAL VARIABLES
*/

// Used to iterate samples array
static uint8 counter = 0;

/***********************************************************************************
* LOCAL FUNCTIONS
*/

/***********************************************************************************
* @fn          timer1_ISR
*
* @brief       ISR for Timer 1. Interrupt occurs when an overflow occurs. Clears
*              flag and feeds a new sample to DSM by writing to T1CC1.
*              CPU interrupt flag (IRCON) for Timer 1 is cleared automatically
*              by hardware
*
* @param       void
*
* @return      void
*/

#pragma vector = T1_VECTOR
__interrupt void timer1_ISR(void)
{
    static uint16 sample = 0;

    // 16 sine samples used as an input to DSM. Calculated like this:
    // 32767 * sin (x * 2*PI/16) for x = 0, 1, ... , 15
    // The samples must be two's complement, so the absolute value of the biggest
    // sample is 0x7FFF (32767).
    static uint16 samples[SAMPLE_SIZE] =
    {
        0x0000, 0x30FB, 0x5A81, 0x7640, 0x7FFF, 0x7640, 0x5A81, 0x30FB,
        0x0000, 0xCF05, 0xA57F, 0x89C0, 0x8001, 0x89C0, 0xA57F, 0xCF05
    };

    // Clear Timer 1 overflow interrupt flag. The clearing has to be like this
    // to avoid loosing other Timer 1 interrupts (writing 1 to T1CTL[7:4] has
    // no effect as these are R/W0 accessible)
    T1CTL = (~T1CTL_OVFIF & 0xF0) | (T1CTL & 0x0F);

    sample = samples[counter];

    // Feed new sample to DSM
    T1CC1L = sample & 0xFF;
    T1CC1H = (sample >> 8) & 0xFF;

    // Increment/Reset sample counter
    if (++counter == SAMPLE_SIZE)
    {
        counter = 0;
    }

}

/***********************************************************************************
* @fn          main
*
* @brief       Uses Timer 1 channel 1 in DSM mode to generate a sine tone
*              at 500 Hz frequency. The sine tone can be listened to through
*              the headphone output on SMARTRF04EB.
*
* @param       none
*
* @return      0
*/


int main(void)
{

    /***************************************************************************
    * Setup clock & frequency
    */

    // Set the system clock source to HS XOSC and max CPU speed,
    // ref. [clk]=>[clk_xosc.c]
    SLEEP &= ~SLEEP_OSC_PD;
    while( !(SLEEP & SLEEP_XOSC_S) );
    CLKCON = (CLKCON & ~(CLKCON_CLKSPD | CLKCON_OSC)) | CLKSPD_DIV_1;
    while (CLKCON & CLKCON_OSC);
    SLEEP |= SLEEP_OSC_PD;

    // Set max tick speed (CLKCON.TICKSPD = 0)
    CLKCON = (CLKCON & ~CLKCON_TICKSPD) | TICKSPD_DIV_1;

    // Clear and set prescaler divider value to 1
    T1CTL = (T1CTL & ~T1CTL_DIV) | T1CTL_DIV_1;


    /***************************************************************************
    * Setup interrupt
    */

    // Clear Timer 1 overflow interrupt flag
    // CPU interrupt flag (IRCON) for Timer 1 is cleared automatically by hardware
    T1CTL &= ~T1CTL_OVFIF;

    // Set interrupt enable for overflow on Timer 1 and disable interrupt for
    // the other channels
    OVFIM = 1;
    T1CCTL0 &= ~T1CCTL0_IM;   // Disable interrupt on channel 0
    T1CCTL1 &= ~T1CCTL1_IM;   // Disable interrupt on channel 1
    T1CCTL2 &= ~T1CCTL2_IM;   // Disable interrupt on channel 2

    // Enable interrupt by setting IEN1.T1IE
    T1IE = 1;

    // Enable global interrupt by setting the IEN0.EA=1
    EA = 1;

    /***************************************************************************
    * Setup peripheral I/O for Timer
    *
    * By choosing alternative 2 location for Timer 1 P1_1 becomes the
    * peripheral pin for DSM channel. This pin is also used as the input
    * to the audio interface on SMARTRF04EB. This way the result of DSM
    * can be listened to through the headphone output on SMARTRF04EB.
    */

    // Select pin 1 on port 1 to be used as peripheral I/O
    P1SEL |= BIT1;

    // Choose alternative 2 location for Timer 1
    PERCFG |= PERCFG_T1CFG;

    // Give priority to Timer 1 over USART 0 for using port 1
    P2SEL |= P2SEL_PRI0P1;


    /***************************************************************************
    * Setup DSM settings
    */

    // Suspend Timer 1
    T1CTL &= ~T1CTL_MODE;

    // Clear Timer counter
    T1CNTL = 0x00;

    // Set the sample rate.
    // This is done by dividing clock tick speed on sample frequency.
    // In this example, we are using tick speed of 26 MHz, and the desired sample
    // rate is 8 kHz. This can be calculated like this: 26000 / 8 = 3250 = 0x0CB1.
    T1CC0H = 0x0C;
    T1CC0L = 0xB1;

    // Set Timer 1 channel 0 to compare mode
    T1CCTL0 |= T1CCTL0_MODE;

    // Load first sample
    T1CC1L = 0x00;
    T1CC1H = 0x00;
    counter++;

    // Set Timer operation to modulo mode
    T1CTL |= T1CTL_MODE_MODULO;

    // Configure DSM by enabling DSM interpolator, output shaping
    // and setting DSM tick speed to 1/8 of clock tick speed. Enable DSM mode.
    T1CCTL1 = (T1CCTL1 & ~T1CCTL1_CAP & ~T1CCTL1_DSM_SPD) | DSM_IP_ON_OS_ON
        | T1C1_DSM_MODE;

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
