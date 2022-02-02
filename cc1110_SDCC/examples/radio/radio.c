#include "../../lib/cc1110.h"
#include "../../lib/cc1110-ext.h"
#include "../../lib/clk_mgmt.h"
#include "../../lib/sdcc_dma.h"
#include "radio.h"
#include "../../lib/interrupt.h"
#include <stdint.h> 

uint8_t radioPktBuffer[PACKET_LENGTH + 3];
uint32_t perRcvdSeqNum = 0;
uint32_t perExpectedSeqNum = 1;
uint32_t perBadPkts = 0;
uint32_t perRcvdPkts = 0;
static uint16_t perRssiOffset;

#define sei() { IEN0 |= IEN0_EA; }

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

        // Determine proper RSSI offset for receiver (freq and rate dependent)
        // and configure the radio frequency to use
        switch (frequency) {
        case FREQUENCY_1_CC1110:    // 915 MHz
            perRssiOffset = 77;
            break;
        case FREQUENCY_2_CC1110:    // 903 MHz
            perRssiOffset = 77;
            break;
        case FREQUENCY_3_CC1110:    // 868 MHz
            perRssiOffset = 77;
            break;
        case FREQUENCY_4_CC1110:    // 433 MHz
        default:
            perRssiOffset = 73;
            break;
        }
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

        // Determine proper RSSI offset for receiver (freq and rate dependent)
        switch (frequency) {
        case FREQUENCY_1_CC1110:    // 915 MHz
            perRssiOffset = 73;
            break;
        case FREQUENCY_2_CC1110:    // 903 MHz
            perRssiOffset = 73;
            break;
        case FREQUENCY_3_CC1110:    // 868 MHz
            perRssiOffset = 73;
            break;
        case FREQUENCY_4_CC1110:    // 433 MHz
        default:
            perRssiOffset = 74;
            break;
        }
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

        // Determine proper RSSI offset for receiver (freq and rate dependent)
        switch (frequency) {
        case FREQUENCY_1_CC1110:    // 915 MHz
            perRssiOffset = 73;
            break;
        case FREQUENCY_2_CC1110:    // 903 MHz
            perRssiOffset = 73;
            break;
        case FREQUENCY_3_CC1110:    // 868 MHz
            perRssiOffset = 73;
            break;
        case FREQUENCY_4_CC1110:    // 433 MHz
        default:
            perRssiOffset = 75;
            break;
        }
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



void pktSetSeqNum(uint32_t seqNum)
{
    // Insert the 4 byte sequence number into a static packet buffer
    radioPktBuffer[3] = (uint8_t) (seqNum>>24);
    radioPktBuffer[4] = (uint8_t) (seqNum>>16);
    radioPktBuffer[5] = (uint8_t) (seqNum>>8);
    radioPktBuffer[6] = (uint8_t) seqNum;
    return;
}

void halWait(uint8_t wait){

   uint32_t largeWait;

   if(wait == 0)
   {return;}

   largeWait = ((uint16_t) (wait << 7));
   largeWait += 59*wait;

   largeWait = (largeWait >> CLKSPD);
   while(largeWait--);

   return;
}

static uint8_t pktCheckId(void)
{
    // The NETWORK_ID_KEY is sent as the second and third byte in the packet
    if ((radioPktBuffer[1]==(uint8_t)(NETWORK_ID_KEY>>8)) &&
        (radioPktBuffer[2]==(uint8_t)NETWORK_ID_KEY)) {

        // Reset the NETWORK_ID_KEY from packet buffer to ensure that the next packet will
        // have to update the buffer with it again (to rule out false positives).
        radioPktBuffer[1] = 0x00;
        radioPktBuffer[2] = 0x00;
        return 1;
    }
    return 0;
}

static uint8_t pktCheckCrc(void)
{
    // Check if CRC_OK bit (bit 7) in the second status byte received is set
    if(radioPktBuffer[PACKET_LENGTH + 2] & 0x80){
        radioPktBuffer[PACKET_LENGTH + 2] = 0x00;   // Clear status byte in buffer
        return 1;
    }
    else {
        return 0;
    }
}

uint32_t pktGetSeqNum(void)
{
    uint32_t seqId = 0;

    // Grab the 4 byte sequence number from static packet buffer
    seqId = ((uint32_t)radioPktBuffer[3]) << 24;
    seqId |= ((uint32_t)radioPktBuffer[4]) << 16;
    seqId |= ((uint32_t)radioPktBuffer[5]) << 8;
    seqId |= ((uint32_t)radioPktBuffer[6]);

    return (seqId);
}

uint16_t convertRssiByte(uint8_t rssiComp)
{
    // Convert RSSI value from 2's complement to decimal value.
    uint16_t rssiDec = (uint16_t) rssiComp;

    // Convert to absolute value (RSSI value from radio has resolution of
    // 0.5 dBm) and subtract the radio's appropriate RSSI offset.
    if(rssiDec < 128){
        return (rssiDec/2) - perRssiOffset;
    }
    else{
        return ((rssiDec - 256)/2) - perRssiOffset;
    }
}

uint8_t pktCheckValidity(void)
{
    // Check if the packet length is correct (byte 0 in packet)
    if (radioPktBuffer[0] == PACKET_LENGTH) {

        // Check if the network identifier bytes matches our network ID (byte 1 + 2)
        if (pktCheckId()) {
            // We have a match, this packet is probably meant for us

            // Check if received packet has correct CRC
            if (pktCheckCrc()) {

                // Extract and check if the packet's sequence number is as estimated
                perRcvdSeqNum = pktGetSeqNum();

                // Check if received packet is the expected packet
                if (perRcvdSeqNum == perExpectedSeqNum) {
                    // Yes, this is the expected packet. Counters are incremented.
                    perExpectedSeqNum++;
                    perRcvdPkts++;
                    return 1;
                }

                // Check if the sequence number is lower than the previous one,
                // if so we will assume a new data burst has started and we
                // will reset our statistics variables.
                else if (perRcvdSeqNum < perExpectedSeqNum) {
                    // Update our expectations assuming this is a new burst
                    perExpectedSeqNum = perRcvdSeqNum + 1;
                    perBadPkts = 0;
                    perRcvdPkts = 1;
                    return 1;
                }

                // For the case of a correct packet, but we have a jump in the
                // sequence numbering meaning some packets in between has been lost.
                // This packet loss will be handled when calculating PER.
                else {
                    perExpectedSeqNum = perRcvdSeqNum + 1;
                    perRcvdPkts++;
                    return 1;
                }
            }
            // For packet with correct ID, but wrong CRC
            else {
                perExpectedSeqNum++;    // Assume the seq num would have been correct
                perBadPkts++;
                perRcvdPkts++;
                return 1;
            }
        }
        // For packets with incorrect network ID, ignore them. If this was caused by
        // e.g. a bit error in one of the network identification bytes this will
        // later be accounted for as a packet error if a following received
        // packet has an unexpectedly high sequence number.
        else {
            return 0;
        }
    }
    // Packets with incorrect length are also ignored.
    else {
        return 0;
    }
}

void rf_IRQ(void) __interrupt RF_VECTOR{
    RFIF &= ~IRQ_DONE;        // Tx/Rx completed, clear interrupt flag
    S1CON &= ~0x03;           // Clear the general RFIF interrupt registers
    
    if (mode == RADIO_MODE_RX) {
        pktRcvdFlag = 1;
    }
    else {
        pktSentFlag = 1;
        RFST = RFST_SIDLE;
       
    }
}

//Millisecond delay
void delayms(int msec){
  for (int x=0;x<msec;x++){ //msec
    for (int y=0;y<500;y++){ //usec
      }
  }
  return;
}

void main(void){
    // Initialize P1_3 and P1_2 for LED output 
    P1SEL &= ~(BIT3);
    P1_3 = 0;
    P1DIR |= (BIT3);
    P1SEL &= ~(BIT2);
    P1_2 = 0;
    P1DIR |= (BIT2);
    uint8_t i;
    uint32_t burstSize;                   // Number of packets to burst
    uint32_t seqNum;                      // Sequence number for TX packet
    //Configure either tx or rx mode 
    mode = RADIO_MODE_TX;
    //mode = RADIO_MODE_RX;

    // Choose the crystal oscillator as the system clock
    SetMainClkSrc(CRYSTAL);

    // Select frequency and data rate from LCD menu, then configure the radio
    
    radioConfigure(DATA_RATE_1_CC1110, FREQUENCY_1_CC1110);

    if (mode == RADIO_MODE_TX) {
        // Set up the DMA to move packet data from buffer to radio
        dmaRadioSetup(RADIO_MODE_TX);

        // Configure interrupt for every time a packet is sent
        HAL_INT_ENABLE(INUM_RF, INT_ON);    // Enable RF general interrupt
        RFIM = IRQ_DONE;                    // Mask IRQ_DONE flag only
        INT_GLOBAL_ENABLE(INT_ON);          // Enable interrupts globally
        
        // Construct the packet to be transmitted in buffer
        radioPktBuffer[0] = PACKET_LENGTH;                  // Length byte
        radioPktBuffer[1] = (uint8_t) (NETWORK_ID_KEY>>8);     // Network identifier
        radioPktBuffer[2] = (uint8_t) NETWORK_ID_KEY;
        // radioPktBuffer[3:6] = 4 byte packet sequence number, written later
        // Fill rest of payload with dummy data. Radio is using data whitening.
        for (i = 7; i <= PACKET_LENGTH; i++) {
            radioPktBuffer[i] = 0xCC;
        }

        burstSize = BURST_SIZE_1;

        while (1) {
            
            // Transmit the packet burst
            for (seqNum = 1; seqNum <= burstSize; seqNum++) {

                // Set correct sequence number to packet
                pktSetSeqNum(seqNum);
                // Send the packet
                P1_2 = 0;
                delayms(500);

                DMAARM |= DMAARM_CHANNEL0;  // Arm DMA channel 0
                RFST = STROBE_TX;           // Switch radio to TX

                RFIF=0;
                P1_2 = 1;

                // Wait until the radio transfer is completed,
                // and then reset pktSentFlag
                while(!pktSentFlag);
                pktSentFlag = 0;

                // Wait approx. 3 ms to let the receiver perform its
                // tasks between each packet
                halWait(3);
            }
        }
    }
    else{

        // Set up the DMA to move packet data from buffer to radio
        dmaRadioSetup(RADIO_MODE_RX);
        // Configure interrupt for every received packet
        HAL_INT_ENABLE(INUM_RF, INT_ON);    // Enable RF general interrupt
        RFIM = IRQ_DONE;                    // Mask IRQ_DONE flag only
        INT_GLOBAL_ENABLE(INT_ON);          // Enable interrupts globally
 
        // Start receiving
        DMAARM = DMAARM_CHANNEL0;           // Arm DMA channel 0
        RFST   = STROBE_RX;                 // Switch radio to RX

        pktRcvdFlag=0;
        while (1) {
            
            // Poll for incoming packet delivered by radio + dma
            if (pktRcvdFlag) {
                pktRcvdFlag = 0;
                // Check if the received packet is a valid PER test packet
                if (pktCheckValidity()) {
                    // Subtract old RSSI value from sum
                    perRssiSum -= perRssiBuf[perRssiBufCounter];
                    // Store new RSSI value in ring buffer, will add it to sum later
                    perRssiBuf[perRssiBufCounter] = convertRssiByte(radioPktBuffer[PACKET_LENGTH+1]);
                }
                delayms(100);
                //if (radioPktBuffer[8] == 0xCC){
                    P1_2^=1;
                //}
                DMAARM = DMAARM_CHANNEL0;
                RFST = RFST_SRX;
            }           
        }
    }
}
