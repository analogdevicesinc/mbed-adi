/**
*   @file     AD5270.h
*   @brief    Header file for AD5270 rheostat
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

#ifndef AD5270_H
#define AD5270_H

#include "mbed.h"

/**
 * @brief Analog Devices AD5270 SPI Digital Rheostat class
 */
class AD5270
{
public:

    /// AD5270 commands
    typedef enum {
        NO_OP               =  0x00,    ///< No data
        NO_OP_cmd           =  0x0000,  ///< 16 bit no data
        WRITE_RDAC          =  0x04,    ///< Write to the RDAC Register
        READ_RDAC           =  0x08,    ///< Read from the RDAC Register
        STORE_50TP          =  0x0C,    ///< Write from RDAC to memory
        SW_RST              =  0x10,    ///< Software reset to last memory location
        READ_50TP_CONTENTS  =  0x14,    ///< Read the last memory contents
        READ_50TP_ADDRESS   =  0x18,    ///< Read the last memory address
        WRITE_CTRL_REG      =  0x1C,    ///< Write to the control Register
        READ_CTRL_REG       =  0x20,    ///< Read from the control Register
        SW_SHUTDOWN         =  0x24,    ///< Software shutdown (0) - Normal, (1) - Shutdown
        HI_Zupper           =  0x80,    ///< Get the SDO line ready for High Z
        HI_Zlower           =  0x01,    ///< Puts AD5270 into High Z mode
        HI_Z_Cmd            =  0x8001   ///< Puts AD5270 into High Z mode*/
    } AD5270Commands_t;

    typedef enum {
        NORMAL_MODE = 0,
        SHUTDOWN_MODE = 1
    } AD5270Modes_t;

    typedef enum {
        PROGRAM_50TP_ENABLE = 1,
        RDAC_WRITE_PROTECT = 2,
        R_PERFORMANCE_ENABLE = 4,
        MEMORY_PROGRAM_SUCCESFUL = 8
    } AD5270ControlRegisterBits_t;

    // SPI configuration & constructor
    AD5270(PinName CS = SPI_CS, float max_resistance = 20000.0, PinName MOSI = SPI_MOSI, PinName MISO = SPI_MISO, PinName SCK = SPI_SCK);
    void frequency(int hz);

    /* RDAC commands */
    void write_ctrl_reg(uint8_t data);
    uint8_t read_ctrl_reg(void);
    void change_mode(AD5270Modes_t mode);
    void set_SDO_HiZ(void);

    /* Wiper R/W methods*/
    void reset_RDAC(void);
    void write_wiper_reg(uint16_t data);
    uint16_t read_wiper_reg(void);


    /* Low level methods */
    uint16_t write_cmd(uint8_t command, uint16_t data = 0x00, bool toggle_cs = true);
    uint16_t write_reg(uint16_t data, bool toggle_cs = true);

    /* Methods that deal with resistance in float format*/
    uint16_t calc_RDAC(float resistance);
    float write_RDAC(float resistance);
    float read_RDAC(void);
    float get_max_resistance(void);

    /* 50 TP methods */
    void enable_50TP_programming(void);
    void store_50TP(void);
    void disable_50TP_programming(void);
    uint8_t read_50TP_last_address(void);
    uint16_t read_50TP_memory(uint8_t address);

    void daisy_chain(uint8_t *buffer, uint8_t size);

    SPI ad5270;    ///< SPI instance of the AD5270
    DigitalOut cs; ///< DigitalOut instance for the chip select of the AD5270

private:
    const static int _RESET = 0xff;
    const static int _REG_OPERATION_TIMEOUT = 2;
    const static int _MEMORY_OPERATION_TIMEOUT = 6;
    const static int _WRITE_OPERATION_50TP_TIMEOUT = 350;
    const static int _DUMMY_BYTE = 0xAA;
    const static uint8_t _SPI_MODE = 1;
    float _max_resistance;
};

#endif
