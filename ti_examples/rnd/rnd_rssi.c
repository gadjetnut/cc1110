/*******************************************************************************
  Filename:    rnd_rssi.c

  Description: This example generates a random number based on an RSSI (radio)
               measurement. The radio settings come from SmartRF® Studio, and
               are the standard settings when "Simple RX" is chosen. The result
               is used to seed the random number generator, which then clocks
               the LSFR once and hence generates a random number.

               The random number is stored as the variable rndNumber which can
               be read in debug mode. Note that the seed may be made more random
               if that is required. Please see comments further down.

               The current way of obtaining the RSSI value may not be robust,
               as there is no guarantee that the radio won't exit RX (the RSSI
               value is not updated if not in RX). However, this should not be a
               major problem, as the RSSI value should be random at the time it
               is stored. See design note DN505 for details on RSSI.

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
// Wait time for RSSI valid, the RF settings in this particular example implies
// 1 msec.
#define RSSI_VALID_TIME  1000


/*******************************************************************************
* LOCAL VARIABLES
*/
static uint32 __xdata rssiValidCnt = 0;



/*******************************************************************************
* LOCAL FUNCTIONS
*/


/*******************************************************************************
* @fn          main
*
* @brief       Sets up the clock as recommended in the datasheet. Puts the radio
*              in RX, samples the RSSI, uses this value as seed and generates a
*              random number.
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


    /* Set up radio. These settings are obtained from SmartRF® Studio.
     * The settings are default values for "Simple RX" mode.
     */
    FSCTRL1   = 0x0A;   // Frequency synthesizer control.
    FSCTRL0   = 0x00;   // Frequency synthesizer control.
    FREQ2     = 0x5D;   // Frequency control word, high byte.
    FREQ1     = 0x93;   // Frequency control word, middle byte.
    FREQ0     = 0xB1;   // Frequency control word, low byte.
    MDMCFG4   = 0x86;   // Modem configuration.
    MDMCFG3   = 0x83;   // Modem configuration.
    MDMCFG2   = 0x03;   // Modem configuration.
    MDMCFG1   = 0x22;   // Modem configuration.
    MDMCFG0   = 0xF8;   // Modem configuration.
    CHANNR    = 0x00;   // Channel number.
    DEVIATN   = 0x44;   // Modem deviation setting (when FSK modulation is enabled).
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
    PKTCTRL0  = 0x04;   // Packet automation control.
    ADDR      = 0x00;   // Device address.
    PKTLEN    = 0xFF;   // Packet length.

    /* Radio setting not from SmartRF® Studio:
     * - Keep the radio in RX.
     * - Auto calibrate when going from IDLE to RX or TX (or FSTXON).
     */
    MCSM1 = (MCSM1 & ~MCSM1_RXOFF_MODE) | MCSM1_RXOFF_MODE_RX;
    MCSM0 = (MCSM0 & ~MCSM0_FS_AUTOCAL) | FS_AUTOCAL_FROM_IDLE;

    /* Radio is now in IDLE.
       Strobe command SRX takes radio to RX. */
    RFST = RFST_SRX;

    /* Wait until RX is entered. */
    while ((MARCSTATE & MARCSTATE_MARC_STATE) != MARC_STATE_RX);

    /* Wait a little longer than expected RSSI response time
     * (see design note DN505). Note that the below loop is inaccurate,
     * so the user must implement a proper delay according to DN505.
     */
    for(rssiValidCnt = 0; rssiValidCnt < RSSI_VALID_TIME; rssiValidCnt++);

    /* Seeding the random generator by writing the RSSI value to the RNDL
     * register (twice). The seed could be made more random, e.g. by combining
     * several RSSI samples and/or not writing the same value twice to RNDL.
     */
    uint8 seed = RSSI;
    RNDL = seed;
    RNDL = seed;

    /* Clock the LFSR once (13x unrolling) to generate pseudo-random bytes. */
    ADCCON1 = (ADCCON1 & ~ADCCON1_RCTRL) | ADCCON1_RCTRL_LFSR13;

    /* Waiting for operation to complete (RCTRL = 00). */
    while (ADCCON1 & ADCCON1_RCTRL_COMPL);

    /* Storing the random number, debug to see the value.
     * Done in two statements to define the order of volatile accesses.
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
