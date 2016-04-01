/**
*   @file     AD5270.cpp
*   @brief    Source file for AD5270 rheostat
*   @author   Analog Devices Inc.
*
* For support please go to:
* Github: https://github.com/analogdevicesinc/mbed-adi
* Support: https://ez.analog.com/community/linux-device-drivers/microcontroller-no-os-drivers
* Product: http://www.analog.com/ad5270
* More: https://wiki.analog.com/resources/tools-software/mbed-drivers-all

********************************************************************************
* Copyright 2016(c) Analog Devices, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*  - Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  - Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
*  - Neither the name of Analog Devices, Inc. nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*  - The use of this software may or may not infringe the patent rights
*    of one or more patent holders.  This license does not release you
*    from the requirement that you obtain separate licenses from these
*    patent holders to use this software.
*  - Use of the software either in source or binary form, must be run
*    on or directly connected to an Analog Devices Inc. component.
*
* THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
********************************************************************************/


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
AD5270::AD5270(PinName CS, float max_resistance, PinName MOSI, PinName MISO, PinName SCK):
    ad5270(MOSI, MISO, SCK), cs(CS), _max_resistance(max_resistance)
{
    ad5270.format(8, 3);
    cs = true;
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
