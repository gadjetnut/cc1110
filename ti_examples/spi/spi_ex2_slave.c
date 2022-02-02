
/***********************************************************************************
  Filename:     spi_ex2_slave.c

  Description:  This example uses a slave to receive data from a master using SPI
                and interrupts.

  Comments:     To execute this example, you can use spi_ex0_master.c in IAR
                project spi0_master_cc2510, as the master. Note that the slave's
                code must be executed before executing the master's code, since
                the slave is clocked by the master. When the buffer is full,
                the red LED on SMARTRF04EB is set. If more data is received
                after this, the new data will overwrite the previous data in buffer.
                You can debug the rxBufferSlave array to verify that the bytes
                received are the correct ones and are in correct order.

                Note:
                Remember to use common ground for the Master and Slave!

***********************************************************************************/

/***********************************************************************************
* INCLUDES
*/
#include <hal_types.h>
#include <hal_defs.h>
#include <hal_cc8051.h>
#include <ioCCxx10_bitdef.h>


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

/***********************************************************************************
* CONSTANTS
*/

// These values will give a baud rate of approx. 1.002930 Mbps for 26 MHz clock
#define SPI_BAUD_M  60
#define SPI_BAUD_E  15

// Define size of buffer
#define BUFFER_SIZE 0x05


/***********************************************************************************
* LOCAL VARIABLES
*/

// Slave's recieve buffer
static uint8 rxBufferSlave[BUFFER_SIZE];

// Set to true when finished recieving. Controls the while-loop in main()
static uint8 done = 0;

/***********************************************************************************
* LOCAL FUNCTIONS
*/

/***********************************************************************************
* @fn          ut0rx_isr
*
* @brief       Interrupt routine which receives data from master
*
* @param       none
*
* @return      0
*/

#pragma vector = URX0_VECTOR
__interrupt void ut0rx_isr(void)
{
    // Clear the CPU URX0IF interrupt flag
    URX0IF = 0;

    static uint8 bufferIndex = 0;

    // Write received byte to buffer
    rxBufferSlave[bufferIndex++] = U0DBUF;

    // if buffer full
    if (bufferIndex == BUFFER_SIZE)
    {
        bufferIndex = 0;
        done = 1;
        P1_3 = 0;   // Set SRF04EB LED3
    }

}
/***********************************************************************************
* @fn          main
*
* @brief       Use SPI interrupts to receive data from master
*
* @param       void
*
* @return      0
*/

int main(void)
{

    /***************************************************************************
     * Setup test stuff
     */
    // Initialize P1_1/3 for SRF04EB LED1/3
    P1SEL &= ~(BIT3 | BIT0);
    P1_0 = 1; P1_3 = 1;
    P1DIR |= (BIT3 | BIT0);

    P1_3 = 1;   // Clear SRF04EB LED3

    /***************************************************************************
     * Setup I/O ports
     *
     * Port and pins used USART0 operating in SPI-mode are
     * MISO (MI): P0_2
     * MOSI (MO): P0_3
     * SSN (SS) : P0_4
     * SCK (C)  : P0_5
     *
     * These pins can be set to function as peripheral I/O so that they
     * can be used by USART0 SPI.
     */

    // Configure USART0 for Alternative 1 => Port P0 (PERCFG.U0CFG = 0)
    // To avoid potential I/O conflict with USART1:
    // configure USART1 for Alternative 2 => Port P1 (PERCFG.U1CFG = 1)
    PERCFG = (PERCFG & ~PERCFG_U0CFG) | PERCFG_U1CFG;

    // Give priority to USART 0 over USART 1 for port 0 pins (default)
    P2DIR = (P2DIR & ~P2DIR_PRIP0) | P2DIR_PRIP0_0;

    // Set function of relevant pins to peripheral I/O (see table 50)
    P0SEL |= (BIT5 | BIT4 | BIT3 | BIT2);


     /***************************************************************************
     * Setup interrupt
     */

    // Clear CPU interrupt flag for USART0 RX (TCON.URX0IF)
    URX0IF = 0;

    // Set individual interrupt enable bit in the peripherals SFR
    // Not any

    // Enable interrupt from USART0 RX by setting [IEN0.URX0IE=1]
    URX0IE = 1;

    // Enable global interrupts
    EA = 1;

    /***************************************************************************
     * Configure SPI
     */

    // Set system clock source to 26 Mhz XSOSC to support maximum transfer speed,
    // ref. [clk]=>[clk_xosc.c]
    SLEEP &= ~SLEEP_OSC_PD;
    while( !(SLEEP & SLEEP_XOSC_S) );
    CLKCON = (CLKCON & ~(CLKCON_CLKSPD | CLKCON_OSC)) | CLKSPD_DIV_1;
    while (CLKCON & CLKCON_OSC);
    SLEEP |= SLEEP_OSC_PD;

    // Set USART to SPI mode and Slave mode
    U0CSR = (U0CSR & ~U0CSR_MODE) | U0CSR_SLAVE;

    // Set:
    // - mantissa value
    // - exponent value
    // - clock phase to be centered on first edge of SCK period
    // - negative clock polarity (SCK low when idle)
    // - bit order for transfers to LSB first
    U0BAUD =  SPI_BAUD_M;
    U0GCR = (U0GCR & ~(U0GCR_BAUD_E | U0GCR_CPOL | U0GCR_CPHA | U0GCR_ORDER))
        | SPI_BAUD_E;

    /***************************************************************************
     * Recieve data
     *
     * An USART0 RX complete interrupt will be generated each time a byte is
     * received. See the interrupt routine (ut0rx_interrupt())
     */

    while(!done)
    {
        // Avoids that while loop is optimized away by compiler
        asm("NOP");
    }

    return 0;
}


/***********************************************************************************
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
***********************************************************************************/

