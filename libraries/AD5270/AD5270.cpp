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
    cs = true;
    ad5270.format(8, _SPI_MODE);
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
 * @brief sets a new value for the RDAC
 * @param resistance new value for the resistance
 * @return actual value of the resistance in the RDAC
 */
float AD5270::write_RDAC(float resistance)
{
    // Compute for the RDAC code nearest to the required feedback resistance
    uint16_t RDAC_val = calc_RDAC(resistance);
    float RDAC_Value = ((static_cast<float> (RDAC_val) * _max_resistance) / 1024.0); // inverse operation to get actual resistance in the RDAC
    write_wiper_reg(RDAC_val);
    return RDAC_Value;
}

/**
 * Reads the RDAC register
 * @return RDAC resistor value
 */
float AD5270::read_RDAC()
{
    uint16_t RDAC_val = read_wiper_reg();
    return ((static_cast<float> (RDAC_val) * _max_resistance) / 1024.0);
}

/**
 *	@brief Puts the AD5270 SDO line in to Hi-Z mode
 *	@return none
 */
void AD5270::set_SDO_HiZ(void)
{
    write_reg(HI_Z_Cmd);
    wait_us(2);
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
uint16_t AD5270::write_cmd(uint8_t command, uint16_t data, bool toggle_cs)
{
    /* build 16 bit data to be written - Command + Value */
    uint16_t ui16Command = ((command & 0x3C) << 8) | (data & 0x3FF);
    return write_reg(ui16Command, toggle_cs);
}

/**
 *  Enables the 50TP memory programming
 */
void AD5270::enable_50TP_programming()
{
    uint8_t regVal = read_ctrl_reg();
    write_cmd(WRITE_CTRL_REG, regVal | PROGRAM_50TP_ENABLE); // RDAC register write protect -  allow update of wiper position through digital interface
}

/**
 *  Stores current RDAC content to the 50TP memory
 */
void AD5270::store_50TP()
{
    write_cmd(STORE_50TP);
    wait_ms(_WRITE_OPERATION_50TP_TIMEOUT);
}

/**
 * Disables the 50TP memory programming
 */
void AD5270::disable_50TP_programming()
{
    uint8_t regVal = read_ctrl_reg();
    write_cmd(WRITE_CTRL_REG, regVal & (~PROGRAM_50TP_ENABLE));
}

/**
 * @brief Writes 16bit data to the AD5270 SPI interface
 * @param data to be written
 * @return data returned by the AD5270
 */
uint16_t AD5270::write_reg(uint16_t data, bool toggle_cs)
{
    uint16_t result;
    uint8_t upper_byte = (data >> 8) & 0xFF;
    uint8_t lower_byte =  data & 0xFF;
    ad5270.format(8, _SPI_MODE);
    cs = false & toggle_cs;
    result  = ((ad5270.write(upper_byte)) << 8);
    result |=   ad5270.write(lower_byte);
    cs = true & toggle_cs;
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

/**
 * Writes the wiper register. This includes reading the control register,
 * setting write protect off, writing the wiper, and reverting the settings
 * to the control reg.
 * @param data to be written
 */
void AD5270::write_wiper_reg(uint16_t data)
{
    uint8_t reg_val = read_ctrl_reg();
    write_cmd(WRITE_CTRL_REG, reg_val | RDAC_WRITE_PROTECT); // RDAC register write protect -  allow update of wiper position through digital interface
    write_cmd(WRITE_RDAC, data); // write data to the RDAC register
    write_cmd(WRITE_CTRL_REG, reg_val); // RDAC register write protect -  allow update of wiper position through digital interface
}

/**
 * Reads the wiper register value
 * @return value of the wiper register
 */
uint16_t AD5270::read_wiper_reg(void)
{
    uint16_t RDAC_val;
    write_cmd(READ_RDAC);
    wait_us(_REG_OPERATION_TIMEOUT);
    RDAC_val = write_cmd(NO_OP);
    return RDAC_val;
}

/**
 * Reads the last programmed value of the 50TP memory
 * @return last programmed value
 */
uint8_t AD5270::read_50TP_last_address(void)
{
    uint8_t ret_val;
    write_cmd(READ_50TP_ADDRESS);
    wait_us(_MEMORY_OPERATION_TIMEOUT);
    ret_val = write_cmd(NO_OP);
    return ret_val;
}

/**
 * Reads the content of a 50TP memory address
 * @param address memory to be read
 * @return value stored in the 50TP address
 */
uint16_t AD5270::read_50TP_memory(uint8_t address)
{
    uint16_t ret_val;
    write_cmd(READ_50TP_CONTENTS, address);
    wait_us(_MEMORY_OPERATION_TIMEOUT);
    ret_val = write_cmd(NO_OP);
    return ret_val;
}

/**
 * Writes the control register
 * @param data to be written
 */
void AD5270::write_ctrl_reg(uint8_t data)
{
    write_cmd(WRITE_CTRL_REG, data);
}

/**
 * Reads the control register
 * @return value of the control register
 */
uint8_t AD5270::read_ctrl_reg(void)
{
    uint8_t ret_val;
    write_cmd(READ_CTRL_REG);
    wait_us(_REG_OPERATION_TIMEOUT);
    ret_val = write_cmd(NO_OP);
    return ret_val;
}

/**
 * Resets the wiper register value to the data last written in the 50TP
 */
void AD5270::reset_RDAC(void)
{
    write_cmd(SW_RST);
}

/**
 * Changes the device mode, enabled or shutdown
 * @param mode - new mode of the device
 */
void AD5270::change_mode(AD5270Modes_t mode)
{
    write_cmd(SW_SHUTDOWN, static_cast<uint8_t>(mode));
}

void AD5270::daisy_chain(uint8_t *buffer, uint8_t size)
{
    cs = 0;
    for(int i = 0; i < size; i++)
        buffer[i] = ad5270.write(buffer[i]);
    cs = 1;
}

