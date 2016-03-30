/**
*   @file     AD5270.cpp
*   @brief    Source file for AD5270 rheostat
*   @version  V0.1
*   @author   ADI
*   @date     March 2015
**/

#include "mbed.h"
#include "AD5270.h"

/**
 * @brief AD5270 constructor, sets CS pin and SPI format
 * @param CS - (optional)chip select of the AD5270
 * @param max_resistance - (optional) nominal resistance of the AD5270 chip in ohms
 * @param MOSI - (optional)pin of the SPI interface
 * @param MISO - (optional)pin of the SPI interface
 * @param SCK  - (optional)pin of the SPI interface
 */
AD5270::AD5270(PinName CS, float max_resistance, PinName MOSI,PinName MISO,PinName SCK):
    ad5270(MOSI,MISO,SCK),cs(CS),_max_resistance(max_resistance)
{
    ad5270.format(8,3);
    cs=true;
}

/**
 * @brief Compute for the nearest RDAC value from given resistance
 * @param resistance - resistor
 * @return RDAC value - closest possible to given resistance
 */
uint16_t AD5270::calc_RDAC(float resistance)
{
    return static_cast<uint16_t>( (resistance / _max_resistance) * 1024.0 );
}

/**
 *	@brief Puts the AD5270 SDO line in to Hi-Z mode
 *	@return none
 */
void AD5270::set_SDO_HiZ(void)
{
    write_reg(HI_Z_Cmd);
    write_reg(NO_OP_cmd);
}

/**
 * @brief Set AD5270 SPI frequency
 * @param hz - SPI bus frequency in hz
 * @return none
 */
void AD5270::frequency(int hz)
{
    ad5270.frequency(hz);
}

/**
 * @brief Issues AD5270 a command
 * @param command - command to be sent
 * @param data - (optional)value for the requested command
 * @return response form the AD5270
 */
uint16_t AD5270::write_cmd(uint8_t command, uint16_t data)
{
    /* build 16 bit data to be written - Command + Value */
    uint16_t ui16Command = ((command & 0x3C) << 8) | (data & 0x3FF);
    return write_reg(ui16Command);
}

/**
 * @brief Writes 16bit data to the AD5270 SPI interface
 * @param data to be written
 * @return data returned by the AD5270
 */
uint16_t AD5270::write_reg(uint16_t data)
{
    uint16_t result;
    uint8_t upper_byte = (data >> 8) & 0xFF;
    uint8_t lower_byte =  data & 0xFF;
    cs = false;
    result  = ((ad5270.write(upper_byte)) << 8);
    result |=   ad5270.write(lower_byte);
    cs = true;
    return result;
}

/**
 * @brief Gets maximum resistance of the AD5270 digital rheostat
 * @return maximum resistance in ohms
 */
float AD5270::get_max_resistance()
{
    return _max_resistance;
}
