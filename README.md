# cc1110
An group of examples for development on CC1110 using SDCC compiler

# Introduction
Texas Instrument CC1110 is a powerful RF "system on a chip" (SOC) that brings together microprocessing power and radio communications all in one extermely low power chip. 

The CC1110 has for years been the workhorse of radio communications found in thousands of commercial applications from remotes, to walkie talkies to hospital equipment. This chip has generally been out of reach to the Maker and University communities due to the expensive development environments. However through this repository you should be able to quickly build RF projects using the free and opensource SDCC compiler.

# CC1110 Features

**Radio**
* High-performance RF transceiver based on the market-leading CC1101
* Excellent receiver selectivity and blocking performance
* High sensitivity (−110* dBm at 1.2 kBaud)
* Programmable data rate up to 500 kBaud
* Programmable output power up to 10 dBm for all supported frequencies
* Frequency range: 300 - 348 MHz, 391 - 464 MHz and 782 - 928 MHz
* Digital RSSI / LQI support

**Low power**
* Low current consumption (RX: 16.2 mA @1.2 kBaud, TX: 15.2 mA @ −6 dBm output power)
* 0.3 μA in PM3 (the operating mode with the lowest power consumption)
* 0.5 µA in PM2 (operating mode with the second lowest power consumption, timer or external interrupt wakeup)

** MCU, Memory, and Peripherals **
* High performance and low power 8051 microcontroller core.
* Powerful DMA functionality
* 32 KB in-system programmable flash and 4 KB RAM
* Full-Speed USB Controller with 1 KB FIFO(CC1111Fx )
* 128-bit AES security coprocessor
* 7 - 12 bit ADC with up to eight inputs
* I2S interface
* Two USARTs
* 16-bit timer with DSM mode
* Three 8-bit timers
* Hardware debug support
* 21 GPIO pins
* SW compatible with CC2510Fx/CC2511Fx

