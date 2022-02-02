/*******************************************************************************
  Filename:    wdt_wd.c

  Description: Uses the WDT in Watchdog mode. Since the Watchdog Timer is never
               cleared, it will cause a reset. The reset source is indicated
               with the LEDs (reset source is obtained from [SLEEP.RST]). The
               LEDs are in no way directly related to the Watchdog Timer, they
               are just used to indicate what is happening.

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

// The LED ON duration
#define LED_ON_TIME  1000



/*******************************************************************************
* LOCAL VARIABLES
*/

static uint32 __xdata ledOnCnt = 0;



/*******************************************************************************
* LOCAL FUNCTIONS
*/
void resetCauseLedIndication(void);



/*******************************************************************************
* @fn          resetCauseLedIndication
*
* @brief       Not directly related to the Watchdog Timer. Blinks green LED
*              or green+yellow LEDs depending on the cause of the last reset.
*              A power-on or a brown-out reset blinks green+yellow LEDs,
*              an external reset blinks the green LED, a Watchdog reset blinks
*              the yellow LED. See the datasheet for details on reset sources.
*
* @param       void
*
* @return      void
*******************************************************************************/
void resetCauseLedIndication()
{
    /* The cause of the last reset can be read from the [SLEEP.RST] bits
     * register. The switch blinks one or both LEDs accordingly.
     */
    switch (SLEEP & SLEEP_RST)
    {
    case SLEEP_RST_POR_BOD:
        P1_0 = 0;   // Set SRF04EB LED1
        P1_3 = 0;   // Set SRF04EB LED3
        for(ledOnCnt = 0; ledOnCnt < LED_ON_TIME; ledOnCnt++);
        P1_0 = 1;   // Clear SRF04EB LED1
        P1_3 = 1;   // Clear SRF04EB LED3
        break;
    case SLEEP_RST_EXT:
        P1_0 = 0;   // Set SRF04EB LED1
        for(ledOnCnt = 0; ledOnCnt < LED_ON_TIME; ledOnCnt++);
        P1_0 = 1;   // Clear SRF04EB LED1
        break;
    case SLEEP_RST_WDT:
        P1_3 = 0;   // Set SRF04EB LED3
        for(ledOnCnt = 0; ledOnCnt < LED_ON_TIME; ledOnCnt++);
        P1_3 = 1;   // Clear SRF04EB LED3
        break;
    default:
        break;
    }
}

/*******************************************************************************
* @fn          main
*
* @brief       Runs the WDT in Watchdog Mode. The Watchdog causes a reset
*              approx. every second. The LEDs indicate what caused the reset
*              (read from [SLEEP.RST]). The LEDs are in no way directly related
*              to the Watchdog Timer, they are just used to indicate what is
*              happening.
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

    /* Sets the Watchdog Timer timeout interval to approx. 1 s. The low power
     * LS RCOSC is used (default); the accuracy of timout interval will vary
     * with the choise of low speed OSC, see the datasheet for details.
     */
    WDCTL = (WDCTL & ~WDCTL_INT) | WDCTL_INT_SEC_1;

    /* Sets the Watchdog Timer in Watchdog mode and starts it. Note that when
     * the WDT is in Watchdog mode, clearing the WDCTL_EN has no effect.
     * Furthermore, if the timeout interval is to be changed, the change should
     * be followed by a clearing of the Watchdog Timer to avoid an unwanted
     * reset. See the datasheet for details. The Watchdog Timer is disabled
     * after a reset.
     */
    WDCTL = (WDCTL & ~WDCTL_MODE) | WDCTL_EN;

    /* Blinks LEDs given the cause of the last reset. */
    resetCauseLedIndication();

    /* Infinite loop.
     */
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
