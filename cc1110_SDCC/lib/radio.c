#include "cc1110-ext.h"
#include "sdcc_dma.h"
#include "clk_mgmt.h"
#include "radio.h"

void dmaRadioSetup(uint8_t mode)
{
    // Some configuration that are common for both TX and RX:

    // CPU has priority over DMA
    // Use 8 bits for transfer count
    // No DMA interrupt when done
    // DMA triggers on radio
    // Single transfer per trigger.
    // One byte is transferred each time.
    dmaConfig.PRIORITY       = DMA_PRI_LOW;
    dmaConfig.M8             = DMA_M8_USE_8_BITS;
    dmaConfig.IRQMASK        = DMA_IRQMASK_DISABLE;
    dmaConfig.TRIG           = DMA_TRIG_RADIO;
    dmaConfig.TMODE          = DMA_TMODE_SINGLE;
    dmaConfig.WORDSIZE       = DMA_WORDSIZE_BYTE;

    if (mode == RADIO_MODE_TX) {
        // Transmitter specific DMA settings

        // Source: radioPktBuffer
        // Destination: RFD register
        // Use the first byte read + 1
        // Sets the maximum transfer count allowed (length byte + data)
        // Data source address is incremented by 1 byte
        // Destination address is constant
        
        SET_WORD(dmaConfig.SRCADDRH, dmaConfig.SRCADDRL, radioPktBuffer);
        SET_WORD(dmaConfig.DESTADDRH, dmaConfig.DESTADDRL, &X_RFD);
        dmaConfig.VLEN           = DMA_VLEN_FIRST_BYTE_P_1;
        SET_WORD(dmaConfig.LENH, dmaConfig.LENL, (PACKET_LENGTH + 1));
        dmaConfig.SRCINC         = DMA_SRCINC_1;
        dmaConfig.DESTINC        = DMA_DESTINC_0;

    }
    else if (mode == RADIO_MODE_RX) {
        // Receiver specific DMA settings:

        // Source: RFD register
        // Destination: radioPktBuffer
        // Use the first byte read + 3 (incl. 2 status bytes)
        // Sets maximum transfer count allowed (length byte + data + 2 status bytes)
        // Data source address is constant
        // Destination address is incremented by 1 byte for each write

        SET_WORD(dmaConfig.SRCADDRH, dmaConfig.SRCADDRL, &X_RFD);
        SET_WORD(dmaConfig.DESTADDRH, dmaConfig.DESTADDRL, radioPktBuffer);

        //dmaConfig.DESTADDRH  = (uint16_t)radioPktBuffer>>8;
        //dmaConfig.DESTADDRL  = (uint16_t)radioPktBuffer;
        //dmaConfig.SRCADDRH   = ((uint16_t)(&X_RFD) >> 8) & 0x00FF;
        //dmaConfig.SRCADDRL   = (uint16_t)(&X_RFD) & 0x00FF;
        //dmaConfig.LENH       = ((PACKET_LENGTH + 3)>>8)&0xFF;
        //dmaConfig.LENL       = (PACKET_LENGTH + 3)&0xFF;

        dmaConfig.VLEN           = DMA_VLEN_FIRST_BYTE_P_3;
        SET_WORD(dmaConfig.LENH, dmaConfig.LENL, (PACKET_LENGTH + 3));
        dmaConfig.SRCINC         = DMA_SRCINC_0;
        dmaConfig.DESTINC        = DMA_DESTINC_1;
    }

    // Save pointer to the DMA configuration struct into DMA-channel 0
    // configuration registers
    SET_WORD(DMA0CFGH, DMA0CFGL, &dmaConfig);

    return;
}

void SetMainClkSrc(uint8_t source)
{
    // source can have the following values:
    // CRYSTAL 0x00  /*  High speed Crystal Oscillator Control */
    // RC      0x01  /*  Low power RC Oscillator */

    if(source) {
      CLKCON |= OSC_BIT;                    // starting the RC Oscillator
      while(!HIGH_FREQUENCY_RC_OSC_STABLE); // waiting until the oscillator is stable
      SLEEP |= OSC_PD_BIT;                  // powering down the unused oscillator
    }
    else {
      SLEEP &= ~OSC_PD_BIT;     // powering down all oscillators
      while(!XOSC_STABLE);      // waiting until the oscillator is stable
      __asm nop __endasm;
      CLKCON &= ~MAIN_OSC_BITS; // starting the Crystal Oscillator
      SLEEP |= OSC_PD_BIT;      // powering down the unused oscillator
    }
}

void radioConfigure(uint32_t dataRate, uint32_t frequency) {

    /* NOTE: The register settings are hard-coded for the predefined set of data
     * rates and frequencies. To enable other data rates or frequencies these
     * register settings should be adjusted accordingly (use SmartRF(R) Studio).
     */

    // Distinguish between chip models (only one revision of each is supported)

    // Set transmit power: +10 dBm (register setting is freq dependent)
    // and configure the radio frequency to use
    switch (frequency) {
    case FREQUENCY_1_CC1110:        // 915 MHz
    default:
        PA_TABLE0 = 0xC0;
        FREQ2 = 0x23;
        FREQ1 = 0x31;
        FREQ0 = 0x3B;
        break;
    case FREQUENCY_2_CC1110:        // 903 MHz
        PA_TABLE0 = 0xC0;
        FREQ2 = 0x22;
        FREQ1 = 0xBB;
        FREQ0 = 0x13;
        break;
    case FREQUENCY_3_CC1110:        // 868 MHz
        PA_TABLE0 = 0xC2;
        FREQ2 = 0x21;
        FREQ1 = 0x62;
        FREQ0 = 0x76;
        break;
    case FREQUENCY_4_CC1110:        // 433 MHz
        PA_TABLE0 = 0xC0;
        FREQ2 = 0x10;               // Actually 433.500 MHz to fit band
        FREQ1 = 0xAC;
        FREQ0 = 0x4E;
        break;
    }

    // Set modulation and other radio parameters
    switch (dataRate) {
    case DATA_RATE_1_CC1110:
        // Settings from SmartRFStudio for CC1110, VERSION == 0x03
        // 250 kBaud, GFSK modulation, 540 kHz RX filter bandwidth.

        // Freq. dependent value: 433 MHz band vs 868/915 MHz band:
        FSCTRL1 = (frequency == FREQUENCY_4_CC1110) ? 0x0C : 0x12;
        FSCTRL1  = 0x12;   // Frequency synthesizer control.
        FSCTRL0  = 0x00;   // Frequency synthesizer control.
        MDMCFG4  = 0x2D;   // Modem configuration.
        MDMCFG3  = 0x3B;   // Modem configuration.
        MDMCFG2  = 0x13;   // Modem configuration.
        MDMCFG1  = 0x22;   // Modem configuration.
        MDMCFG0  = 0xF8;   // Modem configuration.
        DEVIATN  = 0x62;   // Modem deviation setting (when FSK modulation is enabled).
        FREND1   = 0xB6;   // Front end RX configuration.
        FREND0   = 0x10;   // Front end RX configuration.
        MCSM0    = 0x18;   // Main Radio Control State Machine configuration.
        FOCCFG   = 0x1D;   // Frequency Offset Compensation Configuration.
        BSCFG    = 0x1C;   // Bit synchronization Configuration.
        AGCCTRL2 = 0xC7;   // AGC control.
        AGCCTRL1 = 0x00;   // AGC control.
        AGCCTRL0 = 0xB0;   // AGC control.
        FSCAL3   = 0xEA;   // Frequency synthesizer calibration.
        // Freq. dependent value: 433 MHz band vs 868/915 MHz band:
        FSCAL2 = (frequency == FREQUENCY_4_CC1110) ? 0x0A : 0x2A;
        FSCAL0   = 0x1F;   // Frequency synthesizer calibration.
        TEST2    = 0x88;   // Various test settings.
        TEST1    = 0x31;   // Various test settings.
        TEST0    = 0x09;   // Various test settings.
        break;

    case DATA_RATE_2_CC1110:
        // Settings from SmartRFStudio for CC1110, VERSION == 0x03
        // 38.4 kBaud, GFSK modulation, 100 kHz RX filter bandwidth.
        FSCTRL1  = 0x06;   // Frequency synthesizer control.
        FSCTRL0  = 0x00;   // Frequency synthesizer control.
        MDMCFG4  = 0xCA;   // Modem configuration.
        MDMCFG3  = 0x83;   // Modem configuration.
        MDMCFG2  = 0x13;   // Modem configuration.
        MDMCFG1  = 0x22;   // Modem configuration.
        MDMCFG0  = 0xF8;   // Modem configuration.
        DEVIATN  = 0x34;   // Modem deviation setting (when FSK modulation is enabled).
        FREND1   = 0x56;   // Front end RX configuration.
        FREND0   = 0x10;   // Front end RX configuration.
        MCSM0    = 0x18;   // Main Radio Control State Machine configuration.
        FOCCFG   = 0x16;   // Frequency Offset Compensation Configuration.
        BSCFG    = 0x6C;   // Bit synchronization Configuration.
        AGCCTRL2 = 0x43;   // AGC control.
        AGCCTRL1 = 0x40;   // AGC control.
        AGCCTRL0 = 0x91;   // AGC control.
        FSCAL3   = 0xE9;   // Frequency synthesizer calibration.
        // Freq. dependent value: 433 MHz band vs 868/915 MHz band:
        FSCAL2 = (frequency == FREQUENCY_4_CC1110) ? 0x0A : 0x2A;
        FSCAL0   = 0x1F;   // Frequency synthesizer calibration.
        TEST2    = 0x81;   // Various test settings.
        TEST1    = 0x35;   // Various test settings.
        TEST0    = 0x09;   // Various test settings.
        break;

    case DATA_RATE_3_CC1110:
    default:
        // Settings from SmartRFStudio for CC1110, VERSION == 0x03
        // 1.2 kBaud, GFSK modulation, 58 kHz RX filter bandwidth.
        FSCTRL1  = 0x06;   // Frequency synthesizer control.
        FSCTRL0  = 0x00;   // Frequency synthesizer control.
        MDMCFG4  = 0xF5;   // Modem configuration.
        MDMCFG3  = 0x83;   // Modem configuration.
        MDMCFG2  = 0x13;   // Modem configuration.
        MDMCFG1  = 0x22;   // Modem configuration.
        MDMCFG0  = 0xF8;   // Modem configuration.
        DEVIATN  = 0x15;   // Modem deviation setting (when FSK modulation is enabled).
        FREND1   = 0x56;   // Front end RX configuration.
        FREND0   = 0x10;   // Front end RX configuration.
        MCSM0    = 0x18;   // Main Radio Control State Machine configuration.
        FOCCFG   = 0x16;   // Frequency Offset Compensation Configuration.
        BSCFG    = 0x6C;   // Bit synchronization Configuration.
        AGCCTRL2 = 0x03;   // AGC control.
        AGCCTRL1 = 0x40;   // AGC control.
        AGCCTRL0 = 0x91;   // AGC control.
        FSCAL3   = 0xE9;   // Frequency synthesizer calibration.
        // Freq. dependent value: 433 MHz band vs 868/915 MHz band:
        FSCAL2 = (frequency == FREQUENCY_4_CC1110) ? 0x0A : 0x2A;
        FSCAL0   = 0x1F;   // Frequency synthesizer calibration.
        TEST2    = 0x81;   // Various test settings.
        TEST1    = 0x35;   // Various test settings.
        TEST0    = 0x09;   // Various test settings.

        break;
    }


    // Common radio settings for CCxx10, any frequency and data rate
    CHANNR   = 0x00;            // Channel number.
    MCSM1 = 0x30;               // Main Radio Control State Machine configuration.
    IOCFG2 = 0x0B;              // GDO2 output pin configuration.
    IOCFG0 = 0x06;              // GDO0 output pin configuration. Sync word.
    PKTCTRL1 = 0x04;            // Packet automation control.
    PKTCTRL0 = 0x45;            // Packet automation control. Data whitening on.
    ADDR = 0x00;                // Device address. Not used.
    PKTLEN = PACKET_LENGTH;     // Packet length.

    return;
}


