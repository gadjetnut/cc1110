/*
************************************************************************************
I2C Example using i2c_bitbang
Reads the temperature and humidity from an HT21 sensor and outputs to the serial port
************************************************************************************

HTU21 Pins:
SDA=P0_6
SCL=P0_7

Serial port pins:
RX=P0_2
TX=P0_3

Output humidity and temperature format:
H=13.5 T=25.1

*/

#include <stdint.h>
#include <string.h>
#include "../../lib/cc1110-ext.h"
#include "../../lib/radio.h"
#include "../../lib/clk_mgmt.h"
#include "../../lib/interrupt.h"
#include "../../lib/delay.h"
#include "../../lib/sdcc_dma.h"
#include "../../lib/i2c_bitbang.h"
#include <math.h>

const uint16_t POLYNOMIAL = 0x131; //P(x)=x^8+x^5+x^4+1 = 100110001

typedef enum
{
    sht21_ADR_W = 128, // sensor I2C address + write bit
    sht21_ADR_R = 129, // sensor I2C address + read bit
    HTU_ADDRESS = 0x40 << 1,
    BME_ADDRESS = 0x76 << 1
} etI2cHeader;

typedef enum
{
    TRIG_T_MEASUREMENT_HM = 0xE3,    // command trig. temp meas. hold master
    TRIG_RH_MEASUREMENT_HM = 0xE5,   // command trig. humidity meas. hold master
    TRIG_T_MEASUREMENT_POLL = 0xF3,  // command trig. temp meas. no hold master
    TRIG_RH_MEASUREMENT_POLL = 0xF5, // command trig. humidity meas. no hold master
    USER_REG_W = 0xE6,               // command writing user register
    USER_REG_R = 0xE7,               // command reading user register
    SOFT_RESET = 0xFE                // command soft reset
} etSHT2xCommand;

// measurement signal selection
typedef enum
{
    HUMIDITY,
    TEMP
} etSHT2xMeasureType;

// Baudrate = 57.6 kbps (U0BAUD.BAUD_M = 34, U0GCR.BAUD_E = 11)
#define UART_BAUD_M 34
#define UART_BAUD_E 11

// DMA descriptor for UART RX/TX
static DMA_DESC __xdata uartDmaRxTxCh[2];

volatile uint8_t __xdata serialbuffer[20];

void reverse(char __far *str, int __far len)
{
    int __far i = 0, j = len - 1, temp;
    while (i < j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}

int intToStr(int __far x, char __far str[], int __far d)
{
    int __far i = 0;
    while (x)
    {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = 0x00;
    return i;
}

// Converts a floating point number to string.
void ftoa(float __far n, char __far *res, int __far afterpoint)
{
    int __far ipart;
    float __far fpart, fpart2;
    int __far i, neg;

    if (afterpoint > 4)
        afterpoint = 4;

    neg = 0;
    if (n < 0)
    {
        n = n * -1;
        neg = 1;
    }

    ipart = (int)n;
    fpart = n - (float)ipart;
    i = intToStr(ipart, res, 0);

    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.'; // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart2 = fpart * powf(10, afterpoint);
        i = intToStr((int)fpart2, res + i + 1, afterpoint);
        if (neg)
        {
            for (i = strlen(res); i >= 0; i--)
            {
                res[i + 1] = res[i];
            }
            res[0] = '-';
        }
    }
}

uint8_t SHT2x_ReadUserRegister(uint8_t __far *pRegisterValue)
{
    uint8_t __far checksum = 0; //variable for checksum byte
    uint8_t __far error = 0;    //variable for error code
    uint8_t __far i = 0;

    I2c_StartCondition();
    error |= I2c_WriteByte(HTU_ADDRESS); // I2C Adr
    error |= I2c_WriteByte(USER_REG_R);  // Command
    I2c_StartCondition();
    error |= I2c_WriteByte(HTU_ADDRESS | 0x01); // | 0x01 to start read access
    error |= I2c_ReadByte(&*pRegisterValue, NO_ACK, 50);
    I2c_StopCondition();
    return error;
}

uint8_t SHT2x_WriteUserRegister(uint8_t __far *pRegisterValue)
{
    uint8_t __far error = 0; //variable for error code
    I2c_StartCondition();
    error |= I2c_WriteByte(HTU_ADDRESS);
    error |= I2c_WriteByte(USER_REG_W);
    error |= I2c_WriteByte(HTU_ADDRESS);
    error |= I2c_WriteByte(*pRegisterValue);
    I2c_StopCondition();
    return error;
}

uint8_t SHT2x_CheckCrc(uint8_t __far data[], uint8_t __far nbrOfBytes, uint8_t __far checksum)
{
    uint8_t __far crc = 0;
    uint8_t __far byteCtr;
    uint8_t __far bit;
    //calculates 8-Bit checksum with given polynomial
    for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
    {
        crc ^= (data[byteCtr]);
        for (bit = 8; bit > 0; --bit)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ POLYNOMIAL;
            else
                crc = (crc << 1);
        }
    }
    if (crc != checksum)
        return CHECKSUM_ERROR;
    else
        return 0;
}

float SHT2x_CalcRH(uint16_t __far u16sRH)
{
    float __far humidityRH; // variable for result
    u16sRH &= ~0x0003;      // clear bits [1..0] (status bits)
    //-- calculate relative humidity [%RH] --
    humidityRH = -6.0 + 125.0 / 65536 * (float)u16sRH; // RH= -6 + 125 * SRH/2^16
    return humidityRH;
}

float SHT2x_CalcTemperatureC(uint16_t __far u16sT)
{
    float __far temperatureC; // variable for result
    u16sT &= ~0x0003;         // clear bits [1..0] (status bits)
    //-- calculate temperature [°C] --
    temperatureC = -46.85 + 175.72 / 65536 * (float)u16sT; //T= -46.85 + 175.72 * ST/2^16
    return temperatureC;
}

/***********************************************************************************
* @fn          uart0StartTxDmaChan
*
* @brief       Function which sets up a DMA channel for UART0 TX.
*
* @param       DMA_DESC *uartDmaTxDescr - pointer to DMA descriptor for UART TX
*              uint8_t uartDmaTxChan - DMA channel number for UART TX
*              uint8_t* uartTxBuf - pointer to allocated UART TX buffer
*              uint16_t uartTxBufSize - size of allocated UART TX buffer
*
* @return      0
*/
void uart0StartTxDmaChan(DMA_DESC *uartDmaTxDescr,
                         uint8_t uartDmaTxChan,
                         uint8_t *uartTxBuf,
                         uint16_t uartTxBufSize)
{

    // Set source/destination pointer (UART TX buffer address) for UART TX DMA channel,
    // and total number of DMA word transfer (according to UART TX buffer size).
    uartDmaTxDescr->SRCADDRH = (uint16_t)(uartTxBuf + 1) >> 8;
    uartDmaTxDescr->SRCADDRL = (uint16_t)(uartTxBuf + 1);
    uartDmaTxDescr->DESTADDRH = ((uint16_t)(&X_U0DBUF) >> 8) & 0x00FF;
    uartDmaTxDescr->DESTADDRL = (uint16_t)(&X_U0DBUF) & 0x00FF;
    uartDmaTxDescr->LENH = ((uartTxBufSize - 1) >> 8) & 0xFF;
    uartDmaTxDescr->LENL = (uartTxBufSize - 1) & 0xFF;
    uartDmaTxDescr->VLEN = DMA_VLEN_FIXED; // Use fixed length DMA transfer count

    // Perform 1-byte transfers
    uartDmaTxDescr->WORDSIZE = DMA_WORDSIZE_BYTE;

    // Transfer a single word after each DMA trigger
    uartDmaTxDescr->TMODE = DMA_TMODE_SINGLE;

    // DMA word trigger = USARTx TX complete
    uartDmaTxDescr->TRIG = DMA_TRIG_UTX0;

    uartDmaTxDescr->SRCINC = DMA_SRCINC_1;        // Increment source pointer by 1 word
                                                  // address after each transfer.
    uartDmaTxDescr->DESTINC = DMA_DESTINC_0;      // Do not increment destination pointer:
                                                  // points to USART UxDBUF register.
    uartDmaTxDescr->IRQMASK = DMA_IRQMASK_ENABLE; // Enable DMA interrupt to the CPU
    uartDmaTxDescr->M8 = DMA_M8_USE_8_BITS;       // Use all 8 bits for transfer count
    uartDmaTxDescr->PRIORITY = DMA_PRI_LOW;       // DMA memory access has low priority

    // Link DMA descriptor with its corresponding DMA configuration register.
    if (uartDmaTxChan < 1)
    {
        DMA0CFGH = (uint8_t)((uint16_t)uartDmaTxDescr >> 8);
        DMA0CFGL = (uint8_t)((uint16_t)uartDmaTxDescr & 0x00FF);
    }
    else
    {
        DMA1CFGH = (uint8_t)((uint16_t)uartDmaTxDescr >> 8);
        DMA1CFGL = (uint8_t)((uint16_t)uartDmaTxDescr & 0x00FF);
    }

    // Arm the relevant DMA channel for UART TX, and apply 45 NOP's
    // to allow the DMA configuration to load
    DMAARM = ((1 << uartDmaTxChan) & (BIT4 | BIT3 | BIT2 | BIT1 | BIT0));
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;
    __asm nop __endasm;

    // Enable the DMA interrupt (IEN1.DMAIE = IEN0.EA = 1),
    // and clear potential pending DMA interrupt requests (IRCON.DMAIF = 0).
    EA = 1;
    DMAIE = 1;
    DMAIF = 0;

    // Send the very first UART byte to trigger a UART TX session:
    U0DBUF = uartTxBuf[0];

    // At this point the UART peripheral generates a DMA trigger each time it has
    // transmitted a byte, leading to a DMA transfer from the allocated source buffer
    // to the UxDBUF register. Once the DMA controller has completed the defined
    // range of transfers, the CPU vectors its execution to the DMA ISR.
}


int send_to_serial(void)
{
    uint8 n;

    /***************************************************************************
     * Setup I/O ports
     *
     * Port and pins used by USART0 operating in UART-mode are
     * RX     : P0_2
     * TX     : P0_3
     * CT/CTS : P0_4
     * RT/RTS : P0_5
     *
     * These pins can be set to function as peripheral I/O to be be used by UART0.
     * The TX pin on the transmitter must be connected to the RX pin on the receiver.
     * If enabling hardware flow control (U0UCR.FLOW = 1) the CT/CTS (Clear-To-Send)
     * on the transmitter must be connected to the RS/RTS (Ready-To-Send) pin on the
     * receiver.
     */

    // Configure USART0 for Alternative 1 => Port P0 (PERCFG.U0CFG = 0)
    // To avoid potential I/O conflict with USART1:
    // configure USART1 for Alternative 2 => Port P1 (PERCFG.U1CFG = 1)
    PERCFG = (PERCFG & ~PERCFG_U0CFG) | PERCFG_U1CFG;

    // Configure relevant Port P0 pins for peripheral function:
    // P0SEL.SELP0_2/3/4/5 = 1 => RX = P0_2, TX = P0_3, CT = P0_4, RT = P0_5
    P0SEL |= BIT5 | BIT4 | BIT3 | BIT2;

    /***************************************************************************
     * Configure UART
     *
     * The system clock source used is the HS XOSC at 26 MHz speed.
     */

    // Set system clock source to 26 Mhz XSOSC to support maximum transfer speed,
    // ref. [clk]=>[clk_xosc.c]
    SLEEP &= ~SLEEP_OSC_PD;
    while (!(SLEEP & SLEEP_XOSC_S))
        ;
    CLKCON = (CLKCON & ~(CLKCON_CLKSPD | CLKCON_OSC)) | CLKSPD_DIV_1;
    while (CLKCON & CLKCON_OSC)
        ;
    SLEEP |= SLEEP_OSC_PD;

    // Initialise bitrate = 57.6 kbps (U0BAUD.BAUD_M = 34, U0GCR.BAUD_E = 11)
    U0BAUD = UART_BAUD_M;
    U0GCR = (U0GCR & ~U0GCR_BAUD_E) | UART_BAUD_E;

    // Initialise UART protocol (start/stop bit, data bits, parity, etc.):

    // USART mode = UART (U0CSR.MODE = 1)
    U0CSR |= U0CSR_MODE;

    // Start bit level = low => Idle level = high  (U0UCR.START = 0)
    U0UCR &= ~U0UCR_START;

    // Stop bit level = high (U0UCR.STOP = 1)
    U0UCR |= U0UCR_STOP;

    // Number of stop bits = 1 (U0UCR.SPB = 0)
    U0UCR &= ~U0UCR_SPB;

    // Parity = disabled (U0UCR.PARITY = 0)
    U0UCR &= ~U0UCR_PARITY;

    // 9-bit data enable = 8 bits transfer (U0UCR.BIT9 = 0)
    U0UCR &= ~U0UCR_BIT9;

    // Level of bit 9 = 0 (U0UCR.D9 = 0), used when U0UCR.BIT9 = 1
    // Level of bit 9 = 1 (U0UCR.D9 = 1), used when U0UCR.BIT9 = 1
    // Parity = Even (U0UCR.D9 = 0), used when U0UCR.PARITY = 1
    // Parity = Odd (U0UCR.D9 = 1), used when U0UCR.PARITY = 1
    U0UCR &= ~U0UCR_D9;

    // Flow control = disabled (U0UCR.FLOW = 0)
    U0UCR &= ~U0UCR_FLOW;

    // Bit order = LSB first (U0GCR.ORDER = 0)
    U0GCR &= ~U0GCR_ORDER;

    /***************************************************************************
     * Transfer UART data
     */
    for (n=0;n<20;n++){
        if (serialbuffer[n]==0x00){
            break;
        }
    }
    serialbuffer[n-1]=0x0a;//add a line break
    uart0StartTxDmaChan(&uartDmaRxTxCh[1], 1, serialbuffer, n);
    return (0);
}


uint8_t SHT21_MeasureHM(etSHT2xMeasureType eSHT2xMeasureType, uint16_t __far *pMeasurand)

{
    uint8_t __far checksum;  //checksum
    uint8_t __far data[2];   //data array for checksum verification
    uint8_t __far error = 0; //error variable
    uint16_t __far i;        //counting variable
    //-- write I2C sensor address and command --
    I2c_StartCondition();
    error |= I2c_WriteByte(sht21_ADR_W); // I2C Adr
    switch (eSHT2xMeasureType)
    {
    case HUMIDITY:
        error |= I2c_WriteByte(TRIG_RH_MEASUREMENT_HM);
        break;
    case TEMP:
        error |= I2c_WriteByte(TRIG_T_MEASUREMENT_HM);
        break;
    }
    //-- wait until hold master is released --
    I2c_StartCondition();
    error |= I2c_WriteByte(sht21_ADR_R);
    SCL_OPEN();            // set SCL I/O port as input
    for (i = 0; i < 1000; i++) // wait until master hold is released or
    {
        delayMicro(1000); // a timeout (~1s) is reached
        if (SCL_READ() == 1)
            break;
    }
    //-- check for timeout --
    if (SCL_READ() == 0)
        error |= TIME_OUT_ERROR;
    //-- read two data bytes and one checksum byte --
    I2c_ReadByte(&data[0],ACK,50);
    I2c_ReadByte(&data[1],ACK,50);
    *pMeasurand = data[1] | data[0] << 8;
    I2c_ReadByte(&checksum,ACK,50);
    //-- verify checksum --
    error |= SHT2x_CheckCrc(data, 2, checksum);
    I2c_StopCondition();
    return error;
}

uint8_t SHT21_MeasurePoll(etSHT2xMeasureType eSHT2xMeasureType, uint16_t __far *pMeasurand)
{
    uint8_t __far checksum;  //checksum
    uint8_t __far data[2];   //data array for checksum verification
    uint8_t __far error = 0; //error variable
    uint16_t __far i = 0;    //counting variable
    //-- write I2C sensor address and command --
    I2c_StartCondition();
    error |= I2c_WriteByte(sht21_ADR_W); // I2C Adr
    switch (eSHT2xMeasureType)
    {
    case HUMIDITY:
        error |= I2c_WriteByte(TRIG_RH_MEASUREMENT_POLL);
        break;
    case TEMP:
        error |= I2c_WriteByte(TRIG_T_MEASUREMENT_POLL);
        break;
    }
    //-- poll every 10ms for measurement ready. Timeout after 20 retries (200ms)--
    do
    {
        I2c_StartCondition();
        //delayMicro(10000,2); //delay 10ms
        delayMicro(10000); //delay 10ms
        if (i++ >= 20)
            break;
    } while (I2c_WriteByte(sht21_ADR_R) == ACK_ERROR);
    if (i >= 20)
        error |= TIME_OUT_ERROR;
    //-- read two data bytes and one checksum byte --
    I2c_ReadByte(&data[0],ACK, 50);
    I2c_ReadByte(&data[1],ACK, 50);
    I2c_ReadByte(&checksum,NO_ACK, 50);
    *pMeasurand = data[1] | data[0] << 8;
    //-- verify checksum --
    error |= SHT2x_CheckCrc(data, 2, checksum);
    I2c_StopCondition();
    return error;
}

uint8_t GetHTU21(void)
{
    uint8_t __far error = 0, ind = 0;             //variable for error code. For codes see system.h
    uint16_t __far sRH = 0, sT = 0;               //variable for raw humidity ticks
    float __far humidityRH = 0, temperatureC = 0; //variable for relative humidity[%RH] as float
    uint8_t __far userRegister;
    I2c_Init();     //initializes uC-ports for I2C
    delayMicro(15); //wait for sensor initialization t_powerUp (15ms)
    error = 0;      // reset error status
    // --- Reset sensor by command ---
    I2c_StartCondition();
    error |= I2c_WriteByte(HTU_ADDRESS); // I2C Adr
    error |= I2c_WriteByte(SOFT_RESET);  // Command
    I2c_StartCondition();
    error |= I2c_WriteByte(HTU_ADDRESS); // I2C Adr
    error |= I2c_WriteByte(0xE7);        // Command
    I2c_StartCondition();
    error |= I2c_WriteByte(HTU_ADDRESS | 0x01); // | 0x01 to start read access
    error |= I2c_ReadByte(&ind, NO_ACK, 50);

    if (ind != 0x2)
    {
        error = 1;
    }

    delayMicro(15);
    // --- Set Resolution e.g. RH 10bit, Temp 13bit ---
    error |= SHT2x_ReadUserRegister(&userRegister);  //get actual user reg
    error |= SHT2x_WriteUserRegister(&userRegister); //write changed user reg

    // --- measure humidity with "Hold Master Mode (HM)" ---
    if (!error)
        error |= SHT21_MeasureHM(HUMIDITY, &sRH);
    // --- measure temperature with "Polling Mode" (no hold master) ---
    if (!error)
        error |= SHT21_MeasurePoll(TEMP, &sT);
    I2c_StartCondition();
    error |= I2c_WriteByte(HTU_ADDRESS); // I2C Adr
    error |= I2c_WriteByte(SOFT_RESET);  // Command
    I2C_SCL = 0;
    I2C_SDA = 0;
    //-- calculate humidity and temperature --
    if (!error)
        temperatureC = SHT2x_CalcTemperatureC(sT);
    if (!error)
        humidityRH = SHT2x_CalcRH(sRH);

    if (error)
    {
        memcpy(serialbuffer, "ERROR", 5);
        memcpy(serialbuffer + 5, 0x00, 15);
        send_to_serial();
    }
    else
    {
        memcpy(serialbuffer, 0x00, 20);
        serialbuffer[19]=0x00;
        memcpy(serialbuffer, "H=", 2);
        ftoa(humidityRH, serialbuffer + 2, 2);
        memcpy(serialbuffer + 6, " T=", 3);
        ftoa(temperatureC, serialbuffer + 9, 2);
        send_to_serial();
    }
    return (0);
}

main()
{
    strncpy(serialbuffer, "Starting...", 11);
    send_to_serial();
    while (1)
    {
        GetHTU21();
        delayms(2000);
    }
    return (0);
}