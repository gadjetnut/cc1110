#include <stdint.h>
#include <string.h>
#include "../../lib/cc1110-ext.h"
#include "../../lib/radio.h"
#include "../../lib/clk_mgmt.h"
#include "../../lib/interrupt.h"
#include "../../lib/delay.h"

volatile uint8_t __xdata  radioPktBuffer[PACKET_LENGTH + 3];

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

void main(void){
    uint8_t preamble[] = {0x0E, 0xA5, 0x5A};
    uint8_t cnt=48;

    mode = RADIO_MODE_TX;

    /* Initialize P1_3 for LED output */

    P1SEL &= ~(BIT3);
    P1DIR |= (BIT3);
    P1_3 = 0;

    // Choose the crystal oscillator as the system clock
    SetMainClkSrc(CRYSTAL);
    
    // Select frequency and data rate from LCD menu, then configure the radio
    radioConfigure(DATA_RATE_1_CC1110, FREQUENCY_1_CC1110);
    
    // Set up the DMA to move packet data from buffer to radio
    dmaRadioSetup(RADIO_MODE_TX);
    
    // Configure interrupt for every time a packet is sent
    HAL_INT_ENABLE(INUM_RF, INT_ON);    // Enable RF general interrupt
    RFIM = IRQ_DONE;                    // Mask IRQ_DONE flag only
    INT_GLOBAL_ENABLE(INT_ON);          // Enable interrupts globally
    
        // Construct the packet to be transmitted in buffer
    
    radioPktBuffer[0] = PACKET_LENGTH;  // Length byte
    radioPktBuffer[1] = (uint8_t) (NETWORK_ID_KEY>>8); // Network identifier
    radioPktBuffer[2] = (uint8_t) NETWORK_ID_KEY;
    memcpy(radioPktBuffer+3, "Hello", 5);
    radioPktBuffer[8]=cnt;

    while (1){
        DMAARM |= DMAARM_CHANNEL0;  // Arm DMA channel 0
        RFST = STROBE_TX;           // Switch radio to TX
        RFIF=1;
       
        // Wait until the radio transfer is completed,
        // and then reset pktSentFlag
        while(!pktSentFlag);
        pktSentFlag = 0;

        P1_3 ^= 1; //toggle LED
        delayms(10); // Delay 
        radioPktBuffer[8]=cnt++;
        if (cnt==58) cnt=48;

    }
}