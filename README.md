# cc1110
An group of examples for development on CC1110 using SDCC compiler

# Introduction
Texas Instrument CC1110 is a powerful RF "system on a chip" (SOC) that brings together microprocessing power and radio communications all in one extremely low power chip. 

The CC1110 has for years been the workhorse of radio communications found in thousands of commercial applications from remotes, to walkie talkies to hospital equipment. This chip has generally been out of reach to the Maker and University communities due to the expensive development environments. However through this repository you should be able to quickly build RF projects using the free and opensource SDCC compiler.

**Some of the features I love**
* You will struggle to find a microprocessor and RF all on one chip at this price point (~$5 boards on Aliexpress or Ebay). Many of the RF modules out there require an external microprocessor, like an Arduino, but this chip gives you some of the features of microprocessor coupled with industry leading RF capability.  
* Lowest power microprocessor on the market by far. In full sleep mode (three are 3 modes) it consumes only 0.3 μA .. yes that's 0.3
* Excellent documentation. TI is known for its documentation and this is no exception (documentation included in the this repository)
* Military strength 128 bit AES encryption coprocessor that easily encrypts data you are sending over the air and automatically decrypts it at its destination . This is important for security when building IoT devices, for example.
* Ample GPIO pins, hardware SPI interface, hardware ADC, two hardware USARTs (serial ports)
* This repository has full support for the Direct Memory Access (DMA) features of the cc1110. It basically is a hardware feature for moving data from a peripheral to memory and saves alot of CPU processing time. E.g. you can move data really efficiently from the radio to a serial port without needing to buffer it in memory (see Rf2Serial example code)
* There are a ton of interrupts and a number of our examples show you how to use them 

**Some of the things I don't love about cc1110**
* Its based on 8051 architecture (circa 1981 technology) and that means its not the easiest thing to program. On the flip side they have added a ton of built in peripherals (see below). However couple it with a circa 2020 lithium battery to the 1981 CPU and you get ALOT of battery life.
* No opensource debugger. A blinking LED is your new best friend.
* No support for Strings, sprintf etc.. so if you are an Arduino developer then you will go through a period of adjustment to basic c dev :-)

**Repository Details**
I have tried to put together a group a example code that give you the building blocks for any RF based IoT project. I included the TI example code that DOES NOT compile on SDCC but is there for reference if you need more examples.

Here is a list of the examples I developed that are supported by SDCC:
 * Blink - Uses a GPIO output pin to blink an LED and a delay function to delay teh CPU between blinks
 * Helloworld -  Transmits a clear text message "HELLO" every second over RF
 * Radio - Can be compiled as a Transmitter or Receiver and can be used to program two cc1110 chips to communicate from sender to receiver. The receiver blinks the LED whenever it receives a packet from the sender
 * Rf2Serial - Converts incoming RF packet to output to UART (Serial port). You can use the Helloworld or Radio examples to transmit data and RF2Serial to view the data using a serial port monitor 
 * Flash - show you how to store data in the flash memory that persists after power off and read the data back from flash and output the data to a serial port
 * Pm - Is the TI Powermode 3 example converted for SDCC and demonstrates how the chip can be put to sleep an awakened by external interrupt 
 * Sleep - is an example of using a timer to wake the chip from sleep after a time period
 * Uart - is an example on how to output data to one of the chips 2 serial ports
 * MAX7219 - Is an example of how to interface to a SPI compatible device. This example uses the MAX7219 8 digit display.


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
* Wide supply voltage range (2.0V - 3.6V) perfect for unregulated battery power supply
* Hardware based watchdog

**MCU, Memory, and Peripherals**
* High performance and low power 8051 microcontroller core.
* Powerful DMA functionality
* 32 KB in-system programmable flash and 4 KB RAM
* 128-bit AES security coprocessor
* 7 - 12 bit ADC with up to eight inputs
* I2S interface
* Two USARTs
* 16-bit timer with DSM mode
* Three 8-bit timers
* Hardware debug support
* 21 GPIO pins
* SW compatible with CC2510Fx/CC2511Fx

