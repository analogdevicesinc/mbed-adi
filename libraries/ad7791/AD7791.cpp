/**
*   @file     AD7791.cpp
*   @brief    Source file for AD7791 ADC
*   @author   Analog Devices Inc.
*
* For support please go to:
* Github: https://github.com/analogdevicesinc/mbed-adi
* Support: https://ez.analog.com/community/linux-device-drivers/microcontroller-no-os-drivers
* Product: http://www.analog.com/ad7791
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
#include "AD7791.h"

/**
 * @brief AD7791 constructor, sets CS pin and SPI format
 * @param CS - (optional)chip select of the AD7791
 * @param MOSI - (optional)pin of the SPI interface
 * @param MISO - (optional)pin of the SPI interface
 * @param SCK  - (optional)pin of the SPI interface
 */
AD7791::AD7791(float reference_voltage,
               PinName CS,
               PinName MOSI,
               PinName MISO,
               PinName SCK) :
			   miso(MISO), ad7791(MOSI, MISO, SCK), cs(CS),  _vref(reference_voltage)
{
    cs = true; // cs is active low
    ad7791.format(8, _SPI_MODE);
    _continous_conversion = true;
    _channel = DIFFERENTIAL;
}

/**
 * @brief Set AD7791 SPI frequency
 * @param hz - SPI bus frequency in hz
 * @return none
 */
void AD7791::frequency(int hz)
{
    ad7791.frequency(hz);
}

/**
 * @brief Resets the AD7791
 * @return none
 */
void AD7791::reset()
{
    ad7791.format(8, _SPI_MODE);
    cs = false;
    wait_us(_DELAY_TIMING);
    ad7791.write(_RESET);
    ad7791.write(_RESET);
    ad7791.write(_RESET);
    ad7791.write(_RESET);
    wait_us(_DELAY_TIMING);
    cs = true;
    _continous_conversion = true;
}

/**
 * Sets the mode register. Also sets continous mode and range based on the value
 * written in reg_val
 * @param reg_val
 */
void AD7791::write_mode_reg(uint8_t reg_val)
{
    write_reg(MODE_REG, reg_val);
    uint8_t continous_mode = (reg_val & 0xC0);
    if(continous_mode == 0x00) {
        _continous_conversion = true;
    } else {
        _continous_conversion = false;
    }
/*  uint8_t range = (reg_val & 0x30);
    _PGA_gain = 1 << (range >> 4);*/

}

/**
 * Reads the mode register and returns its value
 * @return value of the mode register
 */
uint8_t AD7791::read_mode_reg()
{
    return read_reg(MODE_REG);
}

/**
 * Writes the filter register
 * @param regValue value to be written.
 */
void AD7791::write_filter_reg(uint8_t reg_val)
{
    write_reg(FILTER_REG, reg_val);
}

/**
 * Reads the filter register and returns its value
 * @return the value of the filter register
 */
uint8_t AD7791::read_filter_reg()
{
    return read_reg(FILTER_REG);
}

/**
 * Reads the data register and returns its value
 * @return value of the data register
 */
uint32_t AD7791::read_data_reg()
{
    uint32_t data_result;
    ad7791.format(8, _SPI_MODE);
    cs = false;
    ad7791.write(_DATA_READ | (static_cast<uint8_t>(_channel)));
    data_result  = ((ad7791.write(_DUMMY_BYTE)) << 16);
    data_result |= ((ad7791.write(_DUMMY_BYTE)) << 8 );
    data_result |=  (ad7791.write(_DUMMY_BYTE));
    cs = true;
    return data_result;
}

/**
 * Reads the status register of the ADC and returns its value
 * @return value of the status reg
 */
uint8_t AD7791::read_status_reg()
{
    return read_reg(STATUS_REG);
}


/**
 * @brief Enables/disables continous_conversion mode
 * In Single Conversion mode, read_u16 method will read the MODE register of the ADC,
 * then write the Start single conversion bit and wait for the DOUT/RDY pin to go low,
 * When the pin is driven low, data register is read back from the ADC.
 *
 * In Continous conversion mode, read_u16 method will poll the DOUT/RDY pin, if it is low,
 * the data register is read back from the ADC.
 *
 * @param mode
 * true - continous conversion mode enabled
 * false - single conversion mode enabled
 */
void AD7791::set_conversion_mode(AD7791Mode_t mode)
{
    uint8_t mode_reg_val;
    mode_reg_val = read_mode_reg() & 0x3F;
    mode_reg_val = mode_reg_val | (static_cast<uint8_t>(mode));
    write_mode_reg(mode);
}

/**
 *  - From mbed AnalogIn API -
 * @brief Read the input voltage, represented as an unsigned short in the range [0x0, 0xFFFF]
 * Depending on the conversion mode, this method will have different behavior. Conversion mode is set using
 * set_continous_conversion_mode(bool).
 *
 * In Single Conversion mode, read_u16 method will read the MODE register of the ADC,
 * then write the Start single conversion bit and wait for the DOUT/RDY pin to go low,
 * When the pin is driven low, data register is read back from the ADC.
 *
 * In Continous conversion mode, read_u16 method will poll the DOUT/RDY pin, if it is low,
 * the data register is read back from the ADC.
 *
 * @return 16-bit unsigned short representing the current input voltage, normalised to a 16-bit value
 * returns -1 (0xFFFF) along with a debug message if conversion failed.
 */
uint32_t AD7791::read_u32(void)
{
    uint32_t data_result = 0;
    ad7791.format(8, _SPI_MODE);
    cs = false;
    uint16_t timeout_cnt = 0;
    if(_continous_conversion == false) {

        uint8_t mode_reg = read_mode_reg();
        wait_us(_DELAY_TIMING);

        cs = false;
        mode_reg = (mode_reg & 0x3F) | MD1; // mask single conversion bits
        ad7791.write((MODE_REG << 4) | (static_cast<uint8_t>(_channel))); // start single conversion
        ad7791.write(mode_reg);
        timeout_cnt = _SINGLE_CONVERSION_TIMEOUT; // starts timeout
    } else {
        timeout_cnt = _CONTINOUS_CONVERSION_TIMEOUT; // starts timeout
    }
    wait_us(1);

    while(miso) { // wait for the MISO pin to go low.
        if(timeout_cnt) {
            timeout_cnt--;
        } else {
            cs = true;
#ifdef AD7791_DEBUG_MODE
            printf("timeout occurred reading the AD7791. "); // error, MISO line didn't toggle
#endif
            return -1; // ERROR
        }
        wait_us(10);
    }

    ad7791.write(_DATA_READ | (static_cast<uint8_t>(_channel)));
    data_result  = ((ad7791.write(_DUMMY_BYTE)) << 16);
    data_result |= ((ad7791.write(_DUMMY_BYTE)) << 8 );
    data_result |=  (ad7791.write(_DUMMY_BYTE));
    cs = true;
    return data_result;
}

uint16_t AD7791::read_u16(void)
{
  uint32_t data = read_u32();
  return static_cast<uint16_t>((data & 0xffff00) >> 8);
}

/**
 * @brief Reads a register of the AD7791
 * @param  address - address of the register
 * @return value of the register
 */
uint16_t AD7791::read_reg(AD7791Register_t address)
{
    uint16_t data = address << 12;
    data |= _DUMMY_BYTE;
    data |= _READ_FLAG;
    data |= (static_cast<uint8_t>(_channel) << 8);
    return	write_spi(data);
}

/**
 * @brief Writes a register of the AD7791
 * @param address - address of the register
 * @param reg_val - value to be written
 * @return none
 *
 */
void AD7791::write_reg(AD7791Register_t address, uint8_t reg_val)
{
    uint16_t spi_data = address << 12;
    spi_data |= reg_val;
    spi_data |= (static_cast<uint8_t>(_channel) << 8);
    write_spi(spi_data);
}

/**
 * @brief Writes 16bit data to the AD7791 SPI interface
 * @param reg_val to be written
 * @return data returned by the AD7791
 */
uint16_t AD7791::write_spi(uint16_t reg_val)
{
    uint16_t data_result;
    uint8_t upper_byte = (reg_val >> 8) & 0xFF;
    uint8_t lower_byte = reg_val & 0xFF;
    ad7791.format(8, _SPI_MODE);
    cs = false;
    data_result  =  (ad7791.write(upper_byte) << 8);
    data_result |=   ad7791.write(lower_byte);
    cs = true;
    return data_result;
}

/**
 * Sets the reference voltage of the AD7790
 * @param ref reference voltage to be set
 */
void AD7791::set_reference_voltage(float ref)
{
    _vref = ref;
}

/**
 * Gets the reference voltage of the AD7790
 * @return reference voltage
 */
float AD7791::get_reference_voltage(void)
{
    return _vref;
}

/**
 * Reads the data register of the ADC and converts the result to volts
 * Gain needs to be correctly set using set_gain in order to get accurate results
 * @return voltage of the ADC input
 */
float AD7791::read_voltage(void)
{
    return data_to_voltage(read_u32());
}

/**
 * Converts an uint16_t to voltage.
 * Gain needs to be correctly set using set_gain in order to get accurate results
 * @param data in uint16_t format
 * @return float value of voltage (in V)
 */
float AD7791::data_to_voltage(uint32_t data)
{
    return ((data / static_cast<float>(_RESOLUTION / 2)) - 1) * (_vref );
}

/**
 * Converts voltage to an uint16_t.
 * Gain needs to be correctly set using set_gain in order to get accurate results
 * @param voltage to be converted
 * @return data in uint16_t format
 */
uint32_t AD7791::voltage_to_data(float voltage)
{
    return (((voltage / _vref) + 1)   * static_cast<float>(_RESOLUTION / 2));
}

/**
 * Sets the conversion channel.
 * @param channel
 */
void AD7791::set_channel(AD7791Channel_t channel)
{
    _channel = channel;
}

/**
 *  - From mbed AnalogIn API -
 *  Read the input voltage, represented as a float in the range [0.0, 1.0] - uses the read_u16 method
 *  @returns A floating-point value representing the current input voltage, measured as a percentage
 *  returns 1.0 along with a debug message if the conversion failed
 */
float AD7791::read(void)
{
    float percent;
    uint32_t data;
    data = read_u32();
    percent = (data / static_cast<float>(_RESOLUTION) ); // translate bipolar conversion to [0.0, 1.0] domain
    return percent;
}

#ifdef MBED_OPERATORS

/**
 *  - From mbed AnalogIn API -
 *  An operator shorthand for read()
 *  The float() operator can be used as a shorthand for read() to simplify common code sequences
 */
AD7791::operator float()
{
    return read();
}

#endif



