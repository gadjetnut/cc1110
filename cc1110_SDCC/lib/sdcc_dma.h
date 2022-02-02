#ifndef SDCC_DMA
#define SDCC_DMA
/*==== DECLARATION CONTROL ==================================================*/

/*==== INCLUDES =============================================================*/
#include <stdint.h>

/*==== CONSTS ===============================================================*/


#define DMA_VLEN_USE_LEN                       0x00      // Use LEN for transfer count
#define DMA_VLEN_FIXED               0x00 // Use LEN for transfer count
#define DMA_VLEN_FIRST_BYTE_P_1                0x01      // Transfer the number of bytes specified by the first byte +1
#define DMA_VLEN_FIRST_BYTE                    0x02      // Transfer the number of bytes indicated by the first byte (itself included)
#define DMA_VLEN_FIRST_BYTE_P_2                0x03      // Transfer the number of bytes specified by the first byte +2
#define DMA_VLEN_FIRST_BYTE_P_3                0x04      // Transfer the number of bytes specified by the first byte +3
#define DMA_LEN_MAX                            0xFF      // The maximum length is always decided by the first byte
#define DMA_WORDSIZE_BYTE                      0x00      // Transfer a byte at a time
#define DMA_WORDSIZE_WORD                      0x01      // Transfer a 16-bit word at a time
#define DMA_TMODE_SINGLE                       0x00      // Transfer a single byte/word after each DMA trigger
#define DMA_TMODE_BLOCK                        0x01      // Transfer block of data (length len) after each DMA trigger
#define DMA_TMODE_SINGLE_REPEATED              0x02      // Transfer single byte/word (after len transfers, rearm DMA)
#define DMA_TMODE_BLOCK_REPEATED               0x03      // Transfer block of data (after len transfers, rearm DMA)

#define DMA_TRIG_NONE           0   // No trigger, setting DMAREQ.DMAREQx bit starts transfer
#define DMA_TRIG_PREV           1   // DMA channel is triggered by completion of previous channel
#define DMA_TRIG_T1_CH0         2   // Timer 1, compare, channel 0
#define DMA_TRIG_T1_CH1         3   // Timer 1, compare, channel 1
#define DMA_TRIG_T1_CH2         4   // Timer 1, compare, channel 2
#define DMA_TRIG_T2_COMP        5   // Timer 2, compare
#define DMA_TRIG_T2_OVFL        6   // Timer 2, overflow
#define DMA_TRIG_T3_CH0         7   // Timer 3, compare, channel 0
#define DMA_TRIG_T3_CH1         8   // Timer 3, compare, channel 1
#define DMA_TRIG_T4_CH0         9   // Timer 4, compare, channel 0
#define DMA_TRIG_T4_CH1        10   // Timer 4, compare, channel 1
#define DMA_TRIG_ST            11   // Sleep Timer compare
#define DMA_TRIG_IOC_0         12   // Port 0 I/O pin input transition
#define DMA_TRIG_IOC_1         13   // Port 1 I/O pin input transition
#define DMA_TRIG_URX0          14   // USART0 RX complete
#define DMA_TRIG_UTX0          15   // USART0 TX complete
#define DMA_TRIG_URX1          16   // USART1 RX complete
#define DMA_TRIG_UTX1          17   // USART1 TX complete
#define DMA_TRIG_FLASH         18   // Flash data write complete
#define DMA_TRIG_RADIO         19   // RF packet byte received/transmit
#define DMA_TRIG_ADC_CHALL     20   // ADC end of a conversion in a sequence, sample ready
#define DMA_TRIG_ADC_CH0       21   // ADC end of conversion channel 0 in sequence, sample ready
#define DMA_TRIG_ADC_CH1       22   // ADC end of conversion channel 1 in sequence, sample ready
#define DMA_TRIG_ADC_CH2       23   // ADC end of conversion channel 2 in sequence, sample ready
#define DMA_TRIG_ADC_CH3       24   // ADC end of conversion channel 3 in sequence, sample ready
#define DMA_TRIG_ADC_CH4       25   // ADC end of conversion channel 4 in sequence, sample ready
#define DMA_TRIG_ADC_CH5       26   // ADC end of conversion channel 5 in sequence, sample ready
#define DMA_TRIG_ADC_CH6       27   // ADC end of conversion channel 6 in sequence, sample ready
#define DMA_TRIG_ADC_CH7       28   // ADC end of conversion channel 7 in sequence, sample ready
#define DMA_TRIG_ENC_DW        29   // AES encryption processor requests download input data
#define DMA_TRIG_ENC_UP        30   // AES encryption processor requests upload output data

#define DMA_SRCINC_0                           0x00      // Increment source pointer by 0 bytes/words after each transfer
#define DMA_SRCINC_1                           0x01      // Increment source pointer by 1 bytes/words after each transfer
#define DMA_SRCINC_2                           0x02      // Increment source pointer by 2 bytes/words after each transfer
#define DMA_SRCINC_M1                          0x03      // Decrement source pointer by 1 bytes/words after each transfer

#define DMA_DESTINC_0                          0x00      // Increment destination pointer by 0 bytes/words after each transfer
#define DMA_DESTINC_1                          0x01      // Increment destination pointer by 1 bytes/words after each transfer
#define DMA_DESTINC_2                          0x02      // Increment destination pointer by 2 bytes/words after each transfer
#define DMA_DESTINC_M1                         0x03      // Decrement destination pointer by 1 bytes/words after each transfer

#define DMA_IRQMASK_DISABLE                    0x00      // Disable interrupt generation
#define DMA_IRQMASK_ENABLE                     0x01      // Enable interrupt generation upon DMA channel done

#define DMA_M8_USE_8_BITS                      0x00      // Use all 8 bits for transfer count
#define DMA_M8_USE_7_BITS                      0x01      // Use 7 LSB for transfer count

#define DMA_PRI_LOW                            0x00      // Low, CPU has priority
#define DMA_PRI_GUARANTEED                     0x01      // Guaranteed, DMA at least every second try
#define DMA_PRI_HIGH                           0x02      // High, DMA has priority
#define DMA_PRI_ABSOLUTE                       0x03      // Highest, DMA has priority. Reserved for DMA port access.


/*==== TYPES =================================================================*/

typedef struct {
    uint8_t SRCADDRH;
    uint8_t SRCADDRL;
    uint8_t DESTADDRH;
    uint8_t DESTADDRL;
    uint8_t LENH      : 5;
    uint8_t VLEN      : 3;
    uint8_t LENL      : 8;

    uint8_t TRIG      : 5;
    uint8_t TMODE     : 2;
    uint8_t WORDSIZE  : 1;

    uint8_t PRIORITY  : 2;
    uint8_t M8        : 1;
    uint8_t IRQMASK   : 1;
    uint8_t DESTINC   : 2;
    uint8_t SRCINC    : 2;
} DMA_DESC;

static __xdata DMA_DESC dmaConfig;         // Struct for the DMA configuration

#endif