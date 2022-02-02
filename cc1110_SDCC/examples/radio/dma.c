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