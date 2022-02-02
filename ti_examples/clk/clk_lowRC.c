
/***********************************************************************************
  Filename:     clk_lowrc.c

  Description:  Set 32 kHz clock oscillator to low power RC Oscillator (LS RCOSC).



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


/***********************************************************************************
* LOCAL FUNCTIONS
*/



/***********************************************************************************
* @fn          main
*
* @brief       Set 32 kHz clock oscillator to LP RCOSC.
*
* @param       void
*
* @return      0
*/

int main(void)
{

    /* HS RCOSC must be selected as system clock source before changing
       [CLKCON.OSC32K]. HS RCOSC is selected by default at startup, it is
       thus safe to select the LS RCOSC here. */

    // Change clock source for low power oscillator to LS RCOSC.
    CLKCON |= CLKCON_OSC32;

    /* To calibrate LS RCOSC the system clock source must be changed to HS XOSC.
       Calibration of both LS RCOSC and HS RCOSC (since HS RCOSC is powered up)
       will then be started. The LS RCOSC calibration is performed continuously
       as long as the system clock source is HS XOSC and the device is in active
       or power mode 0. The calibration takes ~2 ms.

       Except from the very first calibration (the first time HS XOSC was set
       as system clock source), the calibration will be aborted if one of the
       following actions are initiated by software:

       a) Switching of clock source from HS XOSC to HS RCOSC
       b) Entering power modes {1-3}

       If one of these actions are initiated during the initial calibration,
       the calibration will complete before the actions take place. During the
       initial calibration, a) or b) can only be initiated once. Anytime
       after the initial calibration (2 ms), if a) or b) are initiated by
       software, it will take appr. 130 us before a) and b) are initiated by
       hardware (and calibration aborts). Therefore, a) and b) must not be
       started again during these 130 us See the datasheet under Low Power
       Crystal Oscillator for more information. */

    // Power up unused oscillator (HS XOSC).
    SLEEP &= ~SLEEP_OSC_PD;
    
    // Wait until the HS XOSC is stable.
    while( !(SLEEP & SLEEP_XOSC_S) );


    // Change the system clock source to HS XOSC.
    CLKCON &= ~CLKCON_OSC;

    // Wait until source has actually changed.
    while (CLKCON & CLKCON_OSC);

    /* Now it will take 2ms before the intial calibration completes. It is
       possible to simply wait 2ms for the calibration to complete, or change
       the clock source again to HS RCOSC and then poll [CLKCON.OSC]. */

    // Change system clock source to HS RCOSC.
    CLKCON |= CLKCON_OSC;

    /* Wait until HS RCOSC and LS RCOSC calibration is complete. This will
       take 2 ms. The system clock source will not change before this
       calibration is complete, because this is the initial calibration.
       If it wasn't, then the calibration would abort. */

    // Wait until system clock source has indeed changed to HS RCOSC.
    while ( !(CLKCON & CLKCON_OSC) );

    /* Now both HS RCOSC and LS RCOSC are calibrated. */

    // Power down the HS XOSC, since it is not beeing used.
    // Note that the HS XOSC should not be powered down before the applied
    // system clock source is stable (SLEEP.HFRC_STB = 1).
    SLEEP |= SLEEP_OSC_PD;

    /* Continue... */

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
