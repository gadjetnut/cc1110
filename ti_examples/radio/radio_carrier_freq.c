/*******************************************************************************
  Filename:    radio_carrier_freq.c

  Description: This example shows how to make the radio transmit an unmodulated
               carrier. The radio settings are calculated with SmartRF® Studio.
               Continuous sending of unmodulated carrier is achieved by setting
               the deviation to zero and setting the radio in TX.

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
* @brief       Sets up the system clock source as recommended in the datasheet.
*              Sets up the radio with settings from SmartRF® Studio. Puts radio
*              in TX, i.e., makes the radio transmit the unmodulated carrier.
*
* @param       void
*
* @return      0
*******************************************************************************/
int main(void)
{
    // Set the system clock source to HS XOSC and max CPU speed,
    // ref. [clk]=>[clk_xosc.c]
    SLEEP &= ~SLEEP_OSC_PD;
    while( !(SLEEP & SLEEP_XOSC_S) );
    CLKCON = (CLKCON & ~(CLKCON_CLKSPD | CLKCON_OSC)) | CLKSPD_DIV_1;
    while (CLKCON & CLKCON_OSC);
    SLEEP |= SLEEP_OSC_PD;


    /* Setup radio with settings from SmartRF® Studio. The default settings are
     * used, except that "unmodulated" is chosen in the "Simple RX tab". This
     * results in an umodulated carrier with a frequency of approx. 2.433 GHz.
     */
    FSCTRL1   = 0x0A;   // Frequency synthesizer control.
    FSCTRL0   = 0x00;   // Frequency synthesizer control.
    FREQ2     = 0x5D;   // Frequency control word, high byte.
    FREQ1     = 0x93;   // Frequency control word, middle byte.
    FREQ0     = 0xB1;   // Frequency control word, low byte.
    MDMCFG4   = 0x86;   // Modem configuration.
    MDMCFG3   = 0x83;   // Modem configuration.
    MDMCFG2   = 0x30;   // Modem configuration.
    MDMCFG1   = 0x22;   // Modem configuration.
    MDMCFG0   = 0xF8;   // Modem configuration.
    CHANNR    = 0x00;   // Channel number.
    DEVIATN   = 0x00;   // Modem deviation setting (when FSK modulation is enabled).
    FREND1    = 0x56;   // Front end RX configuration.
    FREND0    = 0x10;   // Front end RX configuration.
    MCSM0     = 0x14;   // Main Radio Control State Machine configuration.
    FOCCFG    = 0x16;   // Frequency Offset Compensation Configuration.
    BSCFG     = 0x6C;   // Bit synchronization Configuration.
    AGCCTRL2  = 0x03;   // AGC control.
    AGCCTRL1  = 0x40;   // AGC control.
    AGCCTRL0  = 0x91;   // AGC control.
    FSCAL3    = 0xA9;   // Frequency synthesizer calibration.
    FSCAL2    = 0x0A;   // Frequency synthesizer calibration.
    FSCAL1    = 0x00;   // Frequency synthesizer calibration.
    FSCAL0    = 0x11;   // Frequency synthesizer calibration.
    TEST2     = 0x88;   // Various test settings.
    TEST1     = 0x31;   // Various test settings.
    TEST0     = 0x09;   // Various test settings.
    PA_TABLE0 = 0xFE;   // PA output power setting.
    PKTCTRL1  = 0x04;   // Packet automation control.
    PKTCTRL0  = 0x22;   // Packet automation control.
    ADDR      = 0x00;   // Device address.
    PKTLEN    = 0xFF;   // Packet length.

    /* Settings not from SmartRF® Studio. Setting both sync word registers to
     * 0xAA = 0b10101010, i.e., the same as the preamble pattern. Not necessary,
     * but gives control of what the radio attempts to transmit.
     */
    SYNC1     = 0xAA;
    SYNC0     = 0xAA;

    /* Put radio in TX. */
    RFST      = RFST_STX;

    /* Wait for radio to enter TX. */
    while ((MARCSTATE & MARCSTATE_MARC_STATE) != MARC_STATE_TX);

    /* Radio is now in TX. Infinite loop. */
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
