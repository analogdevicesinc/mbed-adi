/**
*   @file     AD7790.h
*   @brief    Header file for AD7790 ADC
*   @version  V0.1
*   @author   ADI
*   @date     March 2015
**/

#ifndef AD7790_H
#define AD7790_H

#include "mbed.h"

/**
 * @brief Analog Devices AD7790 SPI 16-bit Buffered Sigma-Delta ADC
 */
class AD7790
{
public:
    /// AD7790 registers
    typedef enum {
        COMMUNICATION_REG = 0, ///< Communication register
        STATUS_REG = 0,        ///< Status register
        MODE_REG,              ///< Mode register
        FILTER_REG,            ///< Filter Register
        DATA_REG               ///< Data register
    } AD7790Registers_t;

    /// AD7790 channel configuration
    typedef enum {
        DIFFERENTIAL, ///< AIN(+)-AIN(-)
        RESERVED,     ///< reserved
        SHORT,        ///< AIN(-)-AIN(-)
        VDDMONITOR    ///< Monitor VDD
    } AD7790Channel_t;

    AD7790(PinName CS=SPI_CS, PinName MOSI=SPI_MOSI,PinName MISO=SPI_MISO,PinName SCK=SPI_SCK);
    void frequency(int hz);
    void reset(void);
    void write_reg(AD7790Registers_t regAddress, uint8_t regValue);
    uint16_t write_spi(uint16_t data);
    uint16_t read_reg (AD7790Registers_t regAddress);
    uint16_t read_data(void);

    SPI ad7790;    ///< SPI instance of the AD7790
    DigitalOut cs; ///< DigitalOut instance for the chipselect of the AD7790

private:

    const static int _RESET = 0xFF;
    const static int _DUMMY_BYTE = 0xAA;
    const static int _READ_FLAG = 0x0800;
    const static int _DATA_READ    = 0x38;             // Read from the Data Register
};

#endif
