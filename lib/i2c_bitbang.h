#ifndef I2C_BITBANG
#define I2C_BITBANG

#include <stdint.h>
/*
void delayMicro(uint8_t __far microsec);
void set_sda_high(void);
void set_scl_high(void);
void set_sda_low();
void set_scl_low();
void set_sda_conf_low(void);
uint8_t get_sda_conf(void);
uint8_t get_scl_conf(void);
void set_scl_conf_low(void);
void sda_ack(uint8_t ack);
void SDA_OPEN(void);
void SCL_OPEN(void);
void SDA_LOW();
void SCL_LOW();
uint8_t SDA_READ(void);
uint8_t SCL_READ(void);
void I2c_Init ();
static etError I2c_WaitWhileClockStreching(uint8_t __far timeout);
*/

#define I2C_SDA P0_6 //if you change remember to change the config in the set_sda function
#define I2C_SCL P0_7 //if you change remember to change the config in the set_scl function

// I2C acknowledge
typedef enum{
 ACK = 0,
 NO_ACK = 1,
}etI2cAck;

typedef enum{
 NO_ERROR  = 0x00, 
 ACK_ERROR = 0x01,
 TIME_OUT_ERROR = 0x02,
 CHECKSUM_ERROR = 0x04,
 UNIT_ERROR = 0x08
}etError;

void I2c_Init ();
uint8_t I2c_ReadByte(uint8_t __far *rxByte, etI2cAck ack, uint8_t __far timeout);
uint8_t I2c_WriteByte(uint8_t __far txByte);
void I2c_StartCondition();
void delayMicro(uint8_t __far microsec);
void set_sda_high(void);
void set_scl_high(void);
void set_sda_low();
void set_scl_low();
void set_sda_conf_low(void);
uint8_t get_sda_conf(void);
uint8_t get_scl_conf(void);
void set_scl_conf_low(void);
void sda_ack(uint8_t ack);
void SDA_OPEN(void);
void SCL_OPEN(void);
void SDA_LOW();
void SCL_LOW();
uint8_t SDA_READ(void);
uint8_t SCL_READ(void);
void I2c_Init ();
static etError I2c_WaitWhileClockStreching(uint8_t __far timeout);
void I2c_StopCondition(void);


#endif