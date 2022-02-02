/***********************************************************************************
  File name:    adc_extra_int_single.c

  Description:  The example illustrates how the ADC interrupt works, based on
                extra conversion trigger.

                The ADC is set up with the following configuration:
                - Single-ended and single-channel conversion on PIN0_7
                - Reference Voltage is VDD on the AVDD pin
                - 10 bit resolution (256 dec rate)

                The ADC conversion will be triggered as an extra conversion, since
                the ADC interrupt is only triggered upon end of conversion from an
                extra conversion.

                The system source clock source is set to the HS XOSC, with no prescaling
                as recommended in the section "ADC Conversion timing" in the
                datasheet.

                The ADC result is always represented in two's complement form, as
                stated in section "ADC Conversion Result" in the datasheet.
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
* @fn          adc_isr
*
* @brief       ADC Interrupt Service Routine (ISR)
*              The ADC result is read. The ADCIF is automatically cleared by HW
*              when the CPU vectors to the ADC ISR.
*
* @param       none
*
* @return      0
*/

#pragma vector=ADC_VECTOR
__interrupt void adc_isr (void)
{
  /* Store the ADC result from the ADCH/L register to the adc_result variable.
   * The 6 LSBs in ADCL will not contain valid data, and are masked out.
   */
  adc_result = ADCL & 0xC0;
  adc_result |= (ADCH << 8);

}



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


  /* Interrupt configuration :
   * Enable Priority level 2 for group 1, which includes the ADC interrupt.
   * Enable ADC interrupt (ADCIE).
   * Enable interrupts (EA).
   */
  // Set Interrupt group 1 to priority 2
  //  {IP1_IPG1, IP0_IPG1} = 10
  IP1 = BIT1;

  // Set the ADCIE bit
  ADCIE = 1;

  // Set the EA bit
  EA = 1;



  /* I/O-Port configuration :
   * PIN0_7 is configured to an ADC input pin
   */

  // Set [ADCCFG.ADCCFG7 = 1].
  ADCCFG |= ADCCFG_7;



  /* ADC configuration :
   *  - Triggered by Extra conversion
   *  - 10 bit resolution
   *  - Single-ended
   *  - Single-channel, due to only 1 pin is selected in the ADCCFG register
   *  - Reference voltage is VDD on AVDD pin
   *
   * The ADC result is represented in two's complement. The conversion is
   * triggered by writing to the ADCCON3 register
   */


  /* Set [ADCCON2.EREF/EDIV/ECH] bits set according to ADC configuration */
  ADCCON3 = ADCCON3_EREF_AVDD | ADCCON3_EDIV_256 | ADCCON3_ECH_AIN7;



  /* Loop forever
   * The ADC interrupt will trigger when the extra conversion is completed.
   * The interrupt Service Routine will store the conversion result.
   */

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
