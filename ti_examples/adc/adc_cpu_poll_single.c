/***********************************************************************************
  Filename:     adc_cpu_poll_single.c

  Description:  This example performs single-ended, single channel CPU controlled
                ADC conversion.

                The ADC is set up for the following configuration:
                - Single-ended and single-channel conversion on PIN0_7
                - Reference Voltage is VDD on the AVDD pin
                - 12 bit resolution (512 dec rate)

                The ADC conversion will be CPU triggered, and the CPU will
                poll for end of ADC conversion.

                The system clock source is set to the HS XOSC, with no prescaling
                as recommended in the section "ADC Conversion timing" in the
                datasheet.

                The ADC result is always represented in two's complement form,
                as stated in section "ADC Conversion Result" in the datasheet.
                An ADC conversion on a single-ended channel will always be positive.
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
static uint16  adc_result;

/***********************************************************************************
* LOCAL FUNCTIONS
*/



/***********************************************************************************
* @fn          main
*
* @brief       This example performs single-ended, single channel CPU controlled
*              ADC conversion.
*
* @param       none
*
* @return      0
*/

int main(void)
{

  /* Set system clock source to HS XOSC, with no pre-scaling.
   * Ref. [clk]=>[clk_xosc.c]
   */
  SLEEP &= ~SLEEP_OSC_PD;
  while( !(SLEEP & SLEEP_XOSC_S) );
  CLKCON = (CLKCON & ~(CLKCON_CLKSPD | CLKCON_OSC)) | CLKSPD_DIV_1;
  while (CLKCON & CLKCON_OSC);
  SLEEP |= SLEEP_OSC_PD;



  /* I/O-Port configuration :
   * PIN0_7 is configured to an ADC input pin.
   */

  // Set [ADCCFG.ADCCFG7 = 1].
  ADCCFG |= ADCCFG_7;



  /* ADC configuration :
   *  - [ADCCON1.ST] triggered
   *  - 12 bit resolution
   *  - Single-ended
   *  - Single-channel, due to only 1 pin is selected in the ADCCFG register
   *  - Reference voltage is VDD on AVDD pin

   *  Note: - [ADCCON1.ST] must always be written to 11
   *
   *  The ADC result is represented in two's complement.
   */

  // Set [ADCCON1.STSEL] according to ADC configuration */
  ADCCON1 = (ADCCON1 & ~ADCCON1_STSEL) | STSEL_ST | BIT1 | BIT0;

  // Set [ADCCON2.SREF/SDIV/SCH] according to ADC configuration */
  ADCCON2 = ADCCON2_SREF_AVDD | ADCCON2_SDIV_512 | ADCCON2_SCH_AIN7;



  /* ADC conversion :
   * The ADC conversion is triggered by setting [ADCCON1.ST = 1].
   * The CPU will then poll [ADCCON1.EOC] until the conversion is completed.
   */

  /* Set [ADCCON1.ST] and await completion (ADCCON1.EOC = 1) */
  ADCCON1 |= ADCCON1_ST | BIT1 | BIT0;
  while( !(ADCCON1 & ADCCON1_EOC));

  /* Store the ADC result from the ADCH/L register to the adc_result variable.
   * The 4 LSBs in ADCL will not contain valid data, and are masked out.
   */
  adc_result = ADCL & 0xF0;
  adc_result |= (ADCH << 8);


  /* Loop forever */
  while (1);


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
