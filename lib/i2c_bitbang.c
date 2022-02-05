#include <stdint.h>
#include "i2c_bitbang.h"
#include <delay.h>
#include <cc1110.h>
//#include "cc1110-ext.h"

/*
#include <stdint.h>
#include <string.h>

#include "../../lib/radio.h"
#include "../../lib/clk_mgmt.h"
#include "../../lib/interrupt.h"
#include "../../lib/delay.h"
#include "../../lib/sdcc_dma.h"
*/

void set_sda_high(void){
	I2C_SDA=1;
    P0DIR |= (0<<6);  //0=Input;              1=Output; VAL<<PIN
}

void set_scl_high(void){
    I2C_SCL=1;
	P0DIR |= (0<<7);  //0=Input;              1=Output; VAL<<PIN
}

void set_sda_low(){
  I2C_SDA=0;
}

void set_scl_low(){
  I2C_SCL=0;
}

void set_sda_conf_low(void){
	P0DIR |= (1<<6);  //0=Input;              1=Output; VAL<<PIN
    I2C_SDA=0;    
}

uint8_t get_sda_conf(void){
  return(I2C_SDA);
}

uint8_t get_scl_conf(void){
  return(I2C_SCL);
}

void set_scl_conf_low(void){
	P0DIR |= (1<<7);  //0=Input;              1=Output; VAL<<PIN
    I2C_SCL=0;    
}

void sda_ack(uint8_t ack){
    I2C_SDA=ack;
}

void SDA_OPEN(void){
	I2C_SDA=1;
    P0DIR |= (1<<6);  //0=Input;              1=Output; VAL<<PIN
}

void SCL_OPEN(void){
    I2C_SCL=1;
	P0DIR |= (1<<7);  //0=Input;              1=Output; VAL<<PIN
}

void SDA_LOW(){
  I2C_SDA=0;
}

void SCL_LOW(){
  I2C_SCL=0;
}

uint8_t SDA_READ(void){
  return(I2C_SDA);
}

uint8_t SCL_READ(void){
  return(I2C_SCL);
}


void I2c_Init (void)
{
 SDA_OPEN();                  // I2C-bus idle mode SDA released
 SCL_OPEN();                  // I2C-bus idle mode SCL released
}

void I2c_StartCondition (void)
{
  SDA_OPEN();
  delayMicro(1);
  SCL_OPEN();
  delayMicro(1);
  SDA_LOW();
  delayMicro(10);  // hold time start condition (t_HD;STA)
  SCL_LOW();
  delayMicro(10);

}
void I2c_StopCondition (void)
{
  SCL_LOW();
  delayMicro(1);
  SDA_LOW();
  delayMicro(1);
  SCL_OPEN();
  delayMicro(10);  // set-up time stop condition (t_SU;STO)
  SDA_OPEN();
  delayMicro(10);

}

uint8_t I2c_WriteByte(uint8_t __far txByte)
{
  uint8_t error = 0;
  uint8_t mask;
  for(mask = 0x80; mask > 0; mask >>= 1)// shift bit for masking (8 times)
  {
    if((mask & txByte) == 0) SDA_LOW(); // masking txByte, write bit to SDA-Line
    else                     SDA_OPEN();
    delayMicro(1);                    // data set-up time (t_SU;DAT)
    SCL_OPEN();                         // generate clock pulse on SCL
    delayMicro(5);                    // SCL high time (t_HIGH)
    SCL_LOW();
    delayMicro(1);                    // data hold time(t_HD;DAT)
  }
  SDA_OPEN();                           // release SDA-line
  SCL_OPEN();                           // clk #9 for ack
  delayMicro(20);                      // data set-up time (t_SU;DAT)
  if(SDA_READ()) error = ACK_ERROR;     // check ack from i2c slave
  SCL_LOW();
  delayMicro(20);                     // wait to see byte package on scope
  return error;                         // return error code
}

static etError I2c_WaitWhileClockStreching(uint8_t __far timeout)
{
  etError error = NO_ERROR;
  
  while(SCL_READ() == 0)
  {
    if(timeout-- == 0) return TIME_OUT_ERROR;
    delayMicro(1000);
  }
  
  return error;
}

uint8_t I2c_ReadByte(uint8_t __far *rxByte, etI2cAck ack, uint8_t __far timeout)
{
  uint8_t error = 0;
  uint8_t mask;
  *rxByte = 0x00;
  SDA_OPEN();                            // release SDA-line
  for(mask = 0x80; mask > 0; mask >>= 1) // shift bit for masking (8 times)
  { 
    SCL_OPEN();                          // start clock on SCL-line
    delayMicro(1);                     // clock set-up time (t_SU;CLK)
    error = I2c_WaitWhileClockStreching(timeout);// wait while clock streching
    delayMicro(3);                     // SCL high time (t_HIGH)
    if(SDA_READ()) *rxByte |= mask;        // read bit
    SCL_LOW();
    delayMicro(1);                     // data hold time(t_HD;DAT)
  }
  if(ack == ACK) SDA_LOW();              // send acknowledge if necessary
  else           SDA_OPEN();
  delayMicro(1);                       // data set-up time (t_SU;DAT)
  SCL_OPEN();                            // clk #9 for ack
  delayMicro(5);                       // SCL high time (t_HIGH)
  SCL_LOW();
  SDA_OPEN();                            // release SDA-line
  delayMicro(20);                      // wait to see byte package on scope
  return error;                          // return with no error
}
