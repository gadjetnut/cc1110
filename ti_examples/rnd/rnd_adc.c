/*******************************************************************************
  Filename:    rnd_adc.c

  Description: This example generates a random number based on an ADC
               conversion. The ADC is set up for single, low resolution
               conversion on the temperature sensor. The result is used to seed
               the random number generator, which then clocks the LSFR once and
               hence generates a random number.

               The random number is stored as the variable rndNumber which can
               be read in debug mode. Note that the seed may be made more random
               if that is required. Please see comments further down.

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
* @fn          main
*
* @brief       Sets up the clock as recommended in the datasheet. Does an ADC
*              conversion from the temperature sensor value, uses this value as
*              seed and generates a random number.
*
* @param       void
*
* @return      0
*******************************************************************************/
int main(void)
{
    volatile uint16 rndNumber;

    // Set the system clock source to HS XOSC and max CPU speed,
    // ref. [clk]=>[clk_xosc.c]
    SLEEP &= ~SLEEP_OSC_PD;
    while( !(SLEEP & SLEEP_XOSC_S) );
    CLKCON = (CLKCON & ~(CLKCON_CLKSPD | CLKCON_OSC)) | CLKSPD_DIV_1;
    while (CLKCON & CLKCON_OSC);
    SLEEP |= SLEEP_OSC_PD;

    /* Start a single conversion with 1.25 V reference and 9 bit resolution
     * on the temperature sensor.
     */
    ADCCON3 = ADCCON3_EREF_1_25V | ADCCON3_EDIV_128 | ADCCON3_ECH_TEMPR;

    /* Wait for the conversion to end. */
    while (!(ADCCON1 & ADCCON1_EOC));

    /* Storing the 8 LSB of ADC conversion result (see the datasheet for details
     * on the ADCL, ADCH registers). Done in two statements to define the order
     * of volatile accesses.
     */
    uint8 seed = ((ADCL & 0xF0) >> 4);
    seed |= ((ADCH & 0x0F) << 4);

    /* Seeding the random generator by writing the ADC result to the RNLD
     * register (twice). The seed could be made more random, e.g. by combining
     * several ADC samples and/or not writing the same value twice to RNDL.
     */
    RNDL = seed;
    RNDL = seed;

    /* Clock the LFSR once (13x unrolling) to generate pseudo-random bytes. */
    ADCCON1 = (ADCCON1 & ~ADCCON1_RCTRL) | ADCCON1_RCTRL_LFSR13;

    /* Waiting for operation to complete (RCTRL = 00). */
    while (ADCCON1 & ADCCON1_RCTRL_COMPL);

    /* Storing the random number, debug to see the value. Done in two statements
     * to define the order of volatile accesses.
     */
    rndNumber = RNDL;
    rndNumber |= RNDH << 8;

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
