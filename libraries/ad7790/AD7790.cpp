/**
*   @file     AD7790.cpp
*   @brief    Source file for AD7790 ADC
*   @version  V0.1
*   @author   ADI
*   @date     March 2015
**/

#include <stdint.h>
#include "mbed.h"
#include "ad7790.h"


/**
 * @brief AD7790 constructor, sets CS pin and SPI format
 * @param CS - (optional)chip select of the AD7790
 * @param MOSI - (optional)pin of the SPI interface
 * @param MISO - (optional)pin of the SPI interface
 * @param SCK  - (optional)pin of the SPI interface
 */
AD7790::AD7790(PinName CS,
               PinName MOSI,
               PinName MISO,
               PinName SCK) :
    ad7790(MOSI, MISO, SCK),cs(CS)
{
    ad7790.format(8,3);
    cs = true; // cs is active low
}

/**
 * @brief Set AD7790 SPI frequency
 * @param hz - SPI bus frequency in hz
 * @return none
 */
void AD7790::frequency(int hz)
{
    ad7790.frequency(hz);
}

/**
 * @brief Resets the AD7790
 * @return none
 */
void AD7790::reset()
{
    cs = false;
    ad7790.write(_RESET);
    ad7790.write(_RESET);
    ad7790.write(_RESET);
    ad7790.write(_RESET);
    cs = true;
}

/**
 * @brief Reads the data register of the AD7790
 * @return value of the register
 */
uint16_t AD7790::read_data(void)
{
    uint16_t dataResult = 0;

    cs = false;
    ad7790.write(_DATA_READ);
    dataResult  = ((ad7790.write(_DUMMY_BYTE)) << 8);
    dataResult |=  (ad7790.write(_DUMMY_BYTE));
    cs = true;

    return dataResult;
}

/**
 * @brief Reads a register of the AD7790
 * @param  regAddress - address of the register
 * @return value of the register
 */
uint16_t AD7790::read_reg(AD7790Registers_t regAddress)
{
    uint16_t data = regAddress << 12;
    data |= _DUMMY_BYTE;
    data |= _READ_FLAG;
    return	write_spi(data);
}

/**
 * @brief Writes a register of the AD7790
 * @param regAddress - address of the register
 * @param regValue - value to be written
 * @return none
 *
 */
void AD7790::write_reg(AD7790Registers_t regAddress,uint8_t regValue)
{
    uint16_t data = regAddress << 12;
    data |= regValue;
    write_spi(data);
}

/**
 * @brief Writes 16bit data to the AD7790 SPI interface
 * @param data to be written
 * @return data returned by the AD7790
 */
uint16_t AD7790::write_spi(uint16_t data)
{
    uint16_t result;
    uint8_t upper_byte = (data >> 8) & 0xFF;
    uint8_t lower_byte = data & 0xFF;
    cs = false;
    result  =  (ad7790.write(upper_byte) << 8);
    result |=   ad7790.write(lower_byte);
    cs = true;
    return result;
}



