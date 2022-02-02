
/***********************************************************************************
  Filename:     powermode1.c

  Description:  Example describing how to go into Power Mode 1

  Comments:     This code example shows how to correctly enter Power Mode 1 and
                wake up using an External Interrupt. The example uses Button 1 (S1)
                on SMARTRF04EB to generate the External Interrupt.
                For test purpose, LED 1 is used to show when the SoC enters and exits PM1.
                When the LED 1 is set, the SoC will stay in Active Mode for some
                time before entering PM1 again, when the LED is cleared.

                Settings:
                  Interrupt:                        External Interrupt
                  System clock oscillator:          HS XOSC oscillator
                  Clock speed:                      26 Mhz

                Note:
                For some SRF04EB versions P0_0 is connected to the Audio/MIC
                interface through the R9 resistor. In order to avoid unintentional
                signal toggling on P0_0 please remove this resistor.

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

// Wait time in Active mode
#define ACT_MODE_TIME  100000

/***********************************************************************************
* LOCAL VARIABLES
*/

// Variable for active mode duration
static uint32 __xdata activeModeCnt = 0;


/***********************************************************************************
* LOCAL FUNCTIONS
*/

/***********************************************************************************
* @fn          button_isr
*
* @brief       Interrupt Service Routine for P0_1, which is connected to
*              button S1 on SMARTRF04EB. The ISR only clear the interrupt flags
*              and is used to wake up from powermode
*
* @param       void
*
* @return      void
*/

#pragma vector = P0INT_VECTOR
__interrupt void button_isr(void)
{

    /*
     * Note that the order in which the following flags are cleared is important.
     * For level triggered interrupts (port interrupts) one has to clear the module
     * interrupt flag prior to clearing the CPU interrupt flags.
     */

    // Clear interrupt status flag for P0_1
    P0IFG  &= ~BIT1;

    // Clear CPU interrupt flag for P0 (IRCON.P0IF)
    P0IF = 0;

}


/***********************************************************************************
* @fn          main
*
* @brief       Entering Power Mode 1 (PM1) using External Interrupt.
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
     *
     * Setup + enable the interrupt source(s) which is/are intended to wake-up
     * the SoC from PM1. Any SoC peripheral interrupt will wake up the SoC from
     * PM1.
     */

    // Clear CPU interrupt flag for P0 and interrupt status flags for pins 1-3
    P0IF = 0;
    P0IFG &= ~(BIT1 | BIT2 | BIT3);

    // Set individual interrupt enable bit in the peripherals SFR.
    // Set interrupt on rising edge and enable interrupt from pins 1-3 for P0
    PICTL = (PICTL & ~PICTL_P0ICON) | PICTL_P0IENL;

    // Enable P0 interrupts by setting [IEN1.P0IE=1]
    P0IE = 1;

    // Enable global interrupt by setting the [IEN0.EA=1]
    EA = 1;

    /***************************************************************************
     * Setup clock & frequency
     *
     * The system clock source used is the HS XOSC at 26 MHz speed.
     * For PM1 both the HS RCOSC and HS XOSC can be used.
     */

    // Set the system clock source to HS XOSC and max CPU speed,
    // ref. [clk]=>[clk_xosc.c]
    SLEEP &= ~SLEEP_OSC_PD;
    while( !(SLEEP & SLEEP_XOSC_S) );
    CLKCON = (CLKCON & ~(CLKCON_CLKSPD | CLKCON_OSC)) | CLKSPD_DIV_1;
    while (CLKCON & CLKCON_OSC);
    SLEEP |= SLEEP_OSC_PD;

    /************************** Test-loop start *******************************
     *
     * This while-loop is used for testing by looping the process of entering
     * and exiting powermode.  (test purpose)
     */

    do
    {
        /***********************************************************************
         * Setup powermode
         */

        // Select powermode
        // Set [SLEEP.MODE] to PM1.
        SLEEP = (SLEEP & ~SLEEP_MODE) | SLEEP_MODE_PM1;

        P1_0 = 1;   // Clear SRF04EB LED1

        // Enter powermode
        // Apply three NOPs to allow the corresponding interrupt blocking to take
        // effect, before verifying the [SLEEP.MODE] bits below. Note that all
        // interrupts are blocked when [SLEEP.MODE > 0], thus the time between
        // setting [SLEEP.MODE > 0], and asserting [PCON.IDLE] should be as short as
        // possible. If an interrupt occurs before the NOPs have completed, then
        // the enabled ISR shall clear the [SLEEP.MODE] bits
        asm("NOP");
        asm("NOP");
        asm("NOP");

        // If no interrupt was executed in between the above NOPs, then all
        // interrupts are effectively blocked when reaching this code position.

        // If the [SLEEP.MODE] bits have been cleared at this point, which means
        // that an ISR has indeed executed in between the above NOPs, then the
        // application should not enter PM{1 – 3} !
        if (SLEEP & SLEEP_MODE)
        {
            // Set PCON.IDLE to enter the selected PM1.
            PCON |= PCON_IDLE;

            // The SoC is now in PM1 and will only wake up upon any enabled
            // SoC interrupt (push button S1)

            // First instruction upon exiting PM1.
            asm("NOP");
        }

        // Wake up from powermode
        // After waking up, the system clock source is HS RCOSC, but since
        // the system clock source was HS XOSC before entering PM1, the CCxx10
        // will automatically switch the system clock source over to HS XOSC
        // when it becomes ready. We therefore only need to poll until this
        // system clock source has indeed changed to HS XOSC (CLKCON.OSC = 0).
        while ( CLKCON&CLKCON_OSC );

        // Set [SLEEP.OSC_PD=1] to power down the HS RCOSC.
        SLEEP |= SLEEP_OSC_PD;

        // Program is now in Active Mode.

        P1_0 = 0;   // Set SRF04EB LED1

        // Wait some time in Active Mode
        for(activeModeCnt = 0; activeModeCnt < ACT_MODE_TIME; activeModeCnt++);

    } while(1);
    /************************* Test-loop end **********************************/

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
