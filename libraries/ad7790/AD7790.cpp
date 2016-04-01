/**
*   @file     AD7790.cpp
*   @brief    Source file for AD7790 ADC
*   @author   Analog Devices Inc.
*
* For support please go to:
* Github: https://github.com/analogdevicesinc/mbed-adi
* Support: https://ez.analog.com/community/linux-device-drivers/microcontroller-no-os-drivers
* Product: http://www.analog.com/ad7790
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
    ad7790(MOSI, MISO, SCK), cs(CS)
{
    ad7790.format(8, 3);
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
void AD7790::write_reg(AD7790Registers_t regAddress, uint8_t regValue)
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



