/**
*   @file     AD7790.h
*   @brief    Header file for AD7790 ADC
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

    AD7790(PinName CS = SPI_CS, PinName MOSI = SPI_MOSI, PinName MISO = SPI_MISO, PinName SCK = SPI_SCK);
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
