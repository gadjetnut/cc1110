#include <stdint.h>
#include <cc1110.h>
#include "../../lib/cc1110-ext.h"
#include "../../lib/clk_mgmt.h"
#include "max7219.h"

/***********************************************************************************
* CONSTANTS
*/

// These values will give a baud rate of approx. 1.002930 Mbps for 26 MHz clock
#define SPI_BAUD_M  60
#define SPI_BAUD_E  15

#define REG_DECODE        0x09                  
#define REG_INTENSITY     0x0a                 
#define REG_SCAN_LIMIT    0x0b                  
#define REG_SHUTDOWN      0x0c                
#define REG_DISPLAY_TEST  0x0f                     

//Millisecond delay
void delayms(int msec){
  for (int x=0;x<msec;x++){ //msec
    for (int y=0;y<500;y++){ //usec
      }
  }
  return;
}

static void MAX7219_Write (char reg_number, char dataout)
{
  P0_4 = 0;
    // Write byte to USART0 buffer (transmit data)
  U0DBUF = reg_number;

  // Check if byte is transmitted
  while(!(U0CSR & U0CSR_TX_BYTE));

  // Clear transmit byte status
  U0CSR &= ~U0CSR_TX_BYTE;
    // Write byte to USART0 buffer (transmit data)

  U0DBUF = dataout;

  // Check if byte is transmitted
  while(!(U0CSR & U0CSR_TX_BYTE));

  // Clear transmit byte status
  U0CSR &= ~U0CSR_TX_BYTE;
    
  P0_4 = 1;
  }

void displayText(char *str, int len){
  int y=0;
  if (len>8) len=8;
  for (int x=0;x<len;x++){
    if (x<len-1 && str[x+1]=='.'){
      MAX7219_DisplayChar (8-y, str[x], 1);
    }
    else {
      if (str[x]!='.'){
        MAX7219_DisplayChar (8-y, str[x], 0);
      }
      y++;
    }
  }
}

int main(void)
{
    int x;

    /***************************************************************************
     * Setup test stuff
     */
    // Initialize P1_1/3 for SRF04EB LED1/3
    P1SEL &= ~(BIT3 | BIT0);
    P1_0 = 1; P1_3 = 0;
    P1DIR |= (BIT3 | BIT0);

    /***************************************************************************
     * Setup I/O ports
     *
     * Port and pins used by USART0 operating in SPI-mode are
     * MISO (MI): P0_2
     * MOSI (MO): P0_3 
     * SSN (SS) : P0_4
     * SCK (C)  : P0_5
     *
     * These pins can be set to function as peripheral I/O to be be used by
     * USART0 SPI. Note however, when SPI is in master mode, only MOSI, MISO,
     * and SCK should be configured as peripheral I/O's. If the external
     * slave device requires a slave select signal (SSN), then the master
     * can control the external SSN by using one of its GPIO pin as output.
     */

    // Configure USART0 for Alternative 1 => Port P0 (PERCFG.U0CFG = 0)
    // To avoid potential I/O conflict with USART1:
    // configure USART1 for Alternative 2 => Port P1 (PERCFG.U1CFG = 1)
    PERCFG = (PERCFG & ~PERCFG_U0CFG) | PERCFG_U1CFG;

    // Give priority to USART 0 over USART 1 for port 0 pins
    P2DIR = (P2DIR & ~P2DIR_PRIP0) | P2DIR_PRIP0_0;

    // Set pins 2, 3 and 5 as peripheral I/O and pin 4 as GPIO output
    P0SEL = (P0SEL & ~BIT4) | BIT5 | BIT3 | BIT2;
    P0DIR |= BIT4;

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

    // Set USART to SPI mode and Master mode
    U0CSR &= ~(U0CSR_MODE | U0CSR_SLAVE);

    // Set:
    // - mantissa value
    // - exponent value
    // - clock phase to be centered on first edge of SCK period
    // - negative clock polarity (SCK low when idle)
    // - bit order for transfers to LSB first
    U0BAUD = SPI_BAUD_M;
    U0GCR = (U0GCR & ~(U0GCR_BAUD_E)) | SPI_BAUD_E;
    U0GCR |= U0GCR_CPOL | U0GCR_CPHA | U0GCR_ORDER;

    /***************************************************************************
     * Transfer data
     */

    MAX7219_Write(REG_DISPLAY_TEST, 0x00);
    MAX7219_Write(REG_DECODE, 0x00);
    MAX7219_Write(REG_SCAN_LIMIT, 7);
    MAX7219_Write(REG_INTENSITY, 8);
    MAX7219_Write(REG_SHUTDOWN, 0x01);
    MAX7219_Clear ();    
    displayText("HELLO",5);   
    delayms(5000);
    MAX7219_Clear ();
    displayText("25.67C",6);   
    delayms(5000);
    MAX7219_Clear ();
    
    x=0;
    while (1){ 
      P1_3^=1; //Blink the LED
      MAX7219_DisplayChar (1, ' ', 0); 
      for (int y=1;y<9;y++){
        MAX7219_DisplayChar (y, 48+x, 0); //display ascii 0 to 9 and then back to 0
      }
      delayms(500);
      x++;
      if (x==10) x=0;
      }
}

void MAX7219_Clear (void)
{
  char i;
  for (i=1; i < 9; i++)
    MAX7219_Write(i, 0x00);                         
}

unsigned char MAX7219_LookupCode (char character, uint8_t dp)
{
  char i;
  uint8_t d=0;
  if (dp) d=1;  
  if (character>=35 && character<=44) {
    character+=13;
    d=1;
  }
  for (i = 0; MAX7219_Font[i].ascii; i++)             // scan font table for ascii code
    if (character == MAX7219_Font[i].ascii){
      if (d){
        d=MAX7219_Font[i].segs;
        d |= (1<<7);
        return (d);                    
        }
      else
        return MAX7219_Font[i].segs;                    
    }
      
  return 0;                                           
}

//digit = display position 0-7, charachter = what to display, dp = decimal place
void MAX7219_DisplayChar (char digit, char character, uint8_t dp)
{
 MAX7219_Write(digit, MAX7219_LookupCode(character, dp));
}

