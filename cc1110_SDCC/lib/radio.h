#ifndef RADIO
#define RADIO 
#include <cc1110.h>

#define STROBE_TX                    0x03   // Strobe commands for the RFST
#define STROBE_RX                    0x02   // register

#define IRQ_DONE                     0x10   // The IRQ_DONE bit in the RFIF-
                                            // and RFIM-register
#define DMAARM_CHANNEL0              0x01   // The value to arm the DMA
                                            // channel 0 in the DMAARM register

#define NUMBER_OF_MODES              2      // Operational mode constants
#define RADIO_MODE_TX                0x10
#define RADIO_MODE_RX                0x20
// Preset frequency alternatives
#define NUMBER_OF_FREQUENCIES_CC1110  4
#define FREQUENCY_1_CC1110       915000     // kHz. NOTE: If you want to alter
#define FREQUENCY_2_CC1110       903000     // these values you will also have
#define FREQUENCY_3_CC1110       868000     // to modify the register settings
#define FREQUENCY_4_CC1110       433500     // radioConfigure() in
                                            // per_test_radio.c

// Preset data rate alternatives
#define NUMBER_OF_DATA_RATES_CC1110   3
#define DATA_RATE_1_CC1110       250000     // bps. NOTE: If you alter these
#define DATA_RATE_2_CC1110        38400     // values you will also have to
#define DATA_RATE_3_CC1110         1200     // modify register settings in
                                            // radioConfigure() in
                                            // per_test_radio.c
#define PACKET_LENGTH                17     // Payload length. Does not include
                                            // 1 length byte (prefixing payload,
                                            // containing this value) and 2
                                            // appended bytes CRC. Does include
                                            // 2 bytes network identifier and 4
                                            // bytes sequence number, hence
                                            // minimum value is 6.
#define RSSI_AVG_WINDOW_SIZE         32     // Size of ring buffer for RSSI
                                            // values to average over (sliding
                                            // window). Max 256

#define NETWORK_ID_KEY               0x5AA5 // Network ID key that identifies
                                            // transmitter/receiver pair

extern volatile uint8_t __xdata  radioPktBuffer[PACKET_LENGTH + 3];  // Buffer for packets to send or receive,
                                                // sized to match the receiver's needs

static volatile uint8_t pktSentFlag = 0;            // Flag set whenever a packet is sent
static volatile uint8_t pktRcvdFlag = 0;            // Flag set whenever a packet is received
static uint8_t mode;                           // Radio operating mode, either RX or TX
void dmaRadioSetup(uint8_t mode);
void SetMainClkSrc(uint8_t source);
void radioConfigure(uint32_t dataRate, uint32_t frequency);
void clk_wait(uint8_t wait);

#endif