#include <stdint.h>

/*==== CONSTS ================================================================*/

#define PARTNUM_CC1110               0x01   // Part number for the CC1110
#define PARTNUM_CC2510               0x81   // Part number for the CC2510

// Define oldest and newest chip revision supported by radio configuration
#define CC1110_MIN_SUPPORTED_VERSION 0x03   // Oldest CC1110 revision supported
#define CC1110_MAX_SUPPORTED_VERSION 0x03   // Newest CC1110 revision supported
#define CC2510_MIN_SUPPORTED_VERSION 0x04   // Oldest CC2510 revision supported
#define CC2510_MAX_SUPPORTED_VERSION 0x04   // Newest CC2510 revision supported

#define STROBE_TX                    0x03   // Strobe commands for the RFST
#define STROBE_RX                    0x02   // register

#define IRQ_DONE                     0x10   // The IRQ_DONE bit in the RFIF-
                                            // and RFIM-register
#define DMAARM_CHANNEL0              0x01   // The value to arm the DMA
                                            // channel 0 in the DMAARM register

#define NUMBER_OF_MODES              2      // Operational mode constants
#define RADIO_MODE_TX                0x10
#define RADIO_MODE_RX                0x20

#define VERSION_OLD_MENU_LINES       10     // Number of text menu lines used
#define VERSION_NEW_MENU_LINES       9      // for the warnings shown on LCD for
                                            // unsupported chip versions

/* Some adjustable settings */
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


/* Some NOT SO adjustable settings ** See also LOCALS section if manipulated */

// Preset frequency alternatives
#define NUMBER_OF_FREQUENCIES_CC1110  4
#define FREQUENCY_1_CC1110       915000     // kHz. NOTE: If you want to alter
#define FREQUENCY_2_CC1110       903000     // these values you will also have
#define FREQUENCY_3_CC1110       868000     // to modify the register settings
#define FREQUENCY_4_CC1110       433500     // radioConfigure() in
                                            // per_test_radio.c

#define NUMBER_OF_FREQUENCIES_CC2510  4
#define FREQUENCY_1_CC2510      2480000     // kHz. NOTE: If you want to alter
#define FREQUENCY_2_CC2510      2460000     // these values you will also have
#define FREQUENCY_3_CC2510      2440000     // to modify the register settings in
#define FREQUENCY_4_CC2510      2420000     // radioConfigure() in
                                            // per_test_radio.c
// Preset data rate alternatives
#define NUMBER_OF_DATA_RATES_CC1110   3
#define DATA_RATE_1_CC1110       250000     // bps. NOTE: If you alter these
#define DATA_RATE_2_CC1110        38400     // values you will also have to
#define DATA_RATE_3_CC1110         1200     // modify register settings in
                                            // radioConfigure() in
                                            // per_test_radio.c
#define NUMBER_OF_DATA_RATES_CC2510   3
#define DATA_RATE_1_CC2510       500000     // bps. NOTE: If you alter these
#define DATA_RATE_2_CC2510       250000     // values you will also have to
#define DATA_RATE_3_CC2510        10000     // modify register settings in
                                            // radioConfigure() in
                                            // per_test_radio.c

// Preset burst size alternatives, i.e. number of packets to transmit
#define NUMBER_OF_BURST_SIZES         4     // Please update LCD text below
#define BURST_SIZE_1            1000000     // accordingly.
#define BURST_SIZE_2             100000
#define BURST_SIZE_3              10000
#define BURST_SIZE_4               1000

/*==== MACROS ================================================================*/
/*==== TYPES =================================================================*/
/*==== GLOBALS ================================================================*/
/*====  GLOBAL VARS ==========================================================*/

// This is the text displayed on the LCD for each menu option and their mapping
// to the defined values
// Other global variables
extern uint8_t radioPktBuffer[PACKET_LENGTH + 3];  // Buffer for packets to send or receive,
                                                // sized to match the receiver's needs


static const char __code blinkCursor[2] = {'-', '*'};  // Blinking cursor symbols,
                                                       // to indicate link at receiver
static uint8_t blinkCursorIdx = 0;            // Blink counter. Bit5 toggles symbol on LCD.
static uint8_t updateLcd = 0;              // Controls when to update the LCD

static volatile uint8_t pktSentFlag = 0;            // Flag set whenever a packet is sent
static volatile uint8_t pktRcvdFlag = 0;            // Flag set whenever a packet is received
static uint8_t mode;                           // Radio operating mode, either RX or TX

static __xdata DMA_DESC dmaConfig;         // Struct for the DMA configuration

// Receiver variables for PER and RSSI statistics
extern uint32_t perRcvdSeqNum;            // The sequence number of the last
                                        // received packet
extern uint32_t perExpectedSeqNum;        // The expected sequence number of the
                                        // next packet
extern uint32_t perBadPkts;               // The total number of packets received
                                        // with correct ID, but wrong CRC
extern uint32_t perRcvdPkts;              // The total number of received packets
                                        // with correct ID, regardless of CRC
static uint16_t perRssiOffset;             // RSSI offset for receiver, depends on
                                        // chip model and data rate
static uint16_t perRssiBuf[RSSI_AVG_WINDOW_SIZE] = {0};    // Ring buffer for RSSI
                                        // values used for (sliding window)
                                        // averaging
static uint8_t perRssiBufCounter;         // Counter to keep track of the oldest/
                                        // newest byte in RSSI ring buffer
static uint16_t perRssiSum;                // Sum of all RSSI values in buffer,
                                        // as absolute RSSI value

/*==== END OF FILE ==========================================================*/
