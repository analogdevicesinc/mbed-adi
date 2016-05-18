/**
*   @file     AD7791.h
*   @brief    Header file for AD7791 ADC
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

#ifndef AD7791_H
#define AD7791_H

#include "mbed.h"

/**
 * Comment this line if you want to turn off the debug mode.
 * The debug mode will send a message if an exception occurs within AD7791 driver
 */

#define AD7791_DEBUG_MODE

/**
 * @brief Analog Devices AD7791 SPI 16-bit Buffered Sigma-Delta ADC
 */
class AD7791
{
public:
    /// AD7791 registers
    typedef enum {
        COMMUNICATION_REG = 0, ///< Communication register
        STATUS_REG = 0,        ///< Status register
        MODE_REG,              ///< Mode register
        FILTER_REG,            ///< Filter Register
        DATA_REG               ///< Data register
    } AD7791Register_t;

    /// AD7791 channel configuration
    typedef enum {
        DIFFERENTIAL = 0, ///< AIN(+)-AIN(-)
        RESERVED,     ///< reserved
        SHORT,        ///< AIN(-)-AIN(-)
        VDDMONITOR    ///< Monitor VDD
    } AD7791Channel_t;

    typedef enum {
        CONTINOUS_CONVERSION_MODE = 0,
        SINGLE_CONVERSION_MODE = 0x80,
        SHUTDOWN_MODE = 0xC0
    } AD7791Mode_t;

    typedef enum    {
        MD1 = 0x80, ///< Mode Select Bit 1
        MD0 = 0x40, ///< Mode Select Bit 0
//      G1 = 0x20,  ///< Range bit 1
//      G0 = 0x10,  ///< Range bit 0
        BO = 0x08,  ///< Burnout Current Enable bit
        UB = 0x04,  ///< Unipolar/Bipolar bit
        BUF = 0x02, ///< Buffered mode bit
    } ModeRegisterBits_t;

    typedef enum {
        CLKDIV1 = 0x40, ///< Clock divider bit 1
        CLKDIV0 = 0x20, ///< Clock divider bit 0
        FS2 = 0x04, ///< Update rate bit 2
        FS1 = 0x02, ///< Update rate bit 1
        FS0 = 0x01, ///< Update rate bit 0
    } FilterRegisterBits_t;

    /*  typedef enum    {
            RANGE_VREF = 0,
            RANGE_VREF_DIV_2,
            RANGE_VREF_DIV_4,
            RANGE_VREF_DIV_8,
        } AnalogInputRange_t;
    */
    /** SPI configuration & constructor */
    AD7791( float reference_voltage, PinName CS = SPI_CS, PinName MOSI = SPI_MOSI, PinName MISO = SPI_MISO, PinName SCK = SPI_SCK);
    void frequency(int hz);

    /** Low level SPI bus comm methods */
    void reset(void);

    /** Register access methods*/
    void set_channel(AD7791Channel_t channel);
    void set_conversion_mode(AD7791Mode_t mode);
    uint32_t read_data_reg();
    uint8_t read_status_reg(void);
    void write_filter_reg(uint8_t regVal);
    uint8_t read_filter_reg(void);
    void write_mode_reg(uint8_t regVal);
    uint8_t read_mode_reg(void);
//  void  set_range(AnalogInputRange_t range);
//  AnalogInputRange_t get_range(void);

    /** Reference voltage methods */
    void  set_reference_voltage(float ref);
    float get_reference_voltage(void);

    /** Voltage read methods */
    float read_voltage(void);
    float data_to_voltage(uint32_t data);
    uint32_t voltage_to_data(float voltage);

    /** AnalogIn API */
    float read(void);
    uint32_t read_u32(void);
    uint16_t read_u16(void);

#ifdef MBED_OPERATORS
    operator float();
#endif

private:
    DigitalIn miso;
    SPI ad7791;    ///< SPI instance of the AD7791
    DigitalOut cs; ///< DigitalOut instance for the chipselect of the AD7791

    float _vref;
//  uint8_t _PGA_gain;
    bool _continous_conversion;
    AD7791Channel_t _channel;

    void write_reg(AD7791Register_t regAddress, uint8_t regValue);
    uint16_t write_spi(uint16_t data);
    uint16_t read_reg (AD7791Register_t regAddress);

    const static uint16_t _SINGLE_CONVERSION_TIMEOUT = 0xFFFF; // in 10us = 100ms
    const static uint16_t _CONTINOUS_CONVERSION_TIMEOUT = 0xFFFF;
    const static uint32_t _RESOLUTION = 0xFFFFFF;
    const static uint8_t _RESET = 0xFF;
    const static uint8_t _DUMMY_BYTE = 0xFF;
    const static uint16_t _READ_FLAG = 0x0800;
    const static uint8_t _DATA_READ    = 0x38;             // Read from the Data Register
    const static uint8_t _DELAY_TIMING = 0x02;
    const static uint8_t _SPI_MODE = 3;
};

#endif
