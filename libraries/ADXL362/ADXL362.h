#ifndef ADXL362_H_
#define ADXL362_H_

/**
*   @file     ADXL362.cpp
*   @brief    Header file for ADXL362
*   @author   Analog Devices Inc.
*
* For support please go to:
* Github: https://github.com/analogdevicesinc/mbed-adi
* Support: https://ez.analog.com/community/linux-device-drivers/microcontroller-no-os-drivers
* Product: http://www.analog.com/adxl362
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

/**
 * @brief Analog devices ADXL362 Digital Output MEMS Accelerometer
 */
class ADXL362
{
public:

    /* Temperature parameters */
    typedef enum {
        DEVID_AD = 0x00,
        DEVID_MST = 0x01,
        PARTID = 0x02,
        REVID = 0x03,
        XDATA = 0x08,
        YDATA = 0x09,
        ZDATA = 0x0A,
        STATUS = 0x0B,
        FIFO_ENTRIES_L = 0x0C,
        FIFO_ENTRIES_H = 0x0D,
        XDATA_L = 0x0E,
        XDATA_H = 0x0F,
        YDATA_L = 0x10,
        YDATA_H = 0x11,
        ZDATA_L = 0x12,
        ZDATA_H = 0x13,
        TEMP_L = 0x14,
        TEMP_H = 0x15,
        // Reserved = 0x16;
        // Reserved = 0x17;
        SOFT_RESET = 0x1F,
        THRESH_ACT_L = 0x20,
        THRESH_ACT_H = 0x21,
        TIME_ACT = 0x22,
        THRESH_INACT_L = 0x23,
        THRESH_INACT_H = 0x24,
        TIME_INACT_L = 0x25,
        TIME_INACT_H = 0x26,
        ACT_INACT_CTL = 0x27,
        FIFO_CONTROL = 0x28,
        FIFO_SAMPLES = 0x29,
        INTMAP1 = 0x2A,
        INTMAP2 = 0x2B,
        FILTER_CTL = 0x2C,
        POWER_CTL = 0x2D,
        SELF_TEST = 0x2E,
    } ADXL362_register_t;

    typedef enum {
        STANDBY = 0x00,
        MEASUREMENT = 0x02
    } ADXL362_modes_t;

    typedef enum {
        ERR_USER_REGS = 0x80,
        AWAKE = 0x40,
        INACT = 0x20,
        ACT = 0x10,
        FIFO_OVERRUN = 0x08,
        FIFO_WATERMARK = 0x04,
        FIFO_READY = 0x02,
        DATA_READY = 0x01
    } ADXL362_STATUS_reg_bits_t;

    typedef enum {
        LINKLOOP1 = 0x20,
        LINKLOOP0 = 0x10,
        DEFAULTMODE = 0x00,
        LINKED_MODE = 0x10,
        LOOP_MODE = 0x30,
        INACT_REF = 0x08,
        INACT_EN = 0x04,
        ACT_REF = 0x02,
        ACT_EN = 0x01
    } ADXL362_ACT_INACT_CTL_reg_bits_t;

    typedef enum {
        AH = 0x08,
        FIFO_TEMP = 0x04,
        FIFO_MODE1 = 0x02,
        FIFO_MODE = 0x01,
    } ADXL362_FIFO_CONTROL_reg_bits_t;

    typedef enum {
        FIFO_DISABLED = 0x00,
        FIFO_OLDEST = 0x01,
        FIFO_STREAM = 0x02,
        FIFO_TRIGGERED = 0x03,
    } ADXL362_FIFO_modes_t;

    typedef enum {
        INT_LOW = 0x80,
        INT_AWAKE = 0x40,
        INT_INACT = 0x20,
        INT_ACT = 0x10,
        INT_FIFO_OVERRUN = 0x08,
        INT_FIFO_WATERMARK = 0x04,
        INT_FIFO_READY = 0x02,
        INT_DATA_READY = 0x01
    } ADXL362_INTMAP_reg_bits_t;

    typedef enum {
        RANGE1 = 0x80,
        RANGE0 = 0x40,
        RANGE2G = 0x00,
        RANGE4G = 0x40,
        RANGE8G = 0x80,
        HALF_BW = 0x10,
        EXT_SAMPLE = 0x08,
        ODR2 = 0x04,
        ODR1 = 0x02,
        ODR0 = 0x01,
        ODR12HZ = 0x00,
        ODR25HZ = 0x01,
        ODR50Hz = 0x02,
        ODR100HZ = 0x03,
        ODR200Hz = 0x04,
        ODR400HZ = 0x07
    } ADXL362_FILTER_CTL_reg_bits_t;

    typedef enum {
        EXT_CLK = 0x40,
        LOW_NOISE1 = 0x20,
        LOW_NOISE0 = 0x10,
        NORMAL_OPERATION = 0x00,
        LOW_NOISE = 0x10,
        ULTRALOW_NOISE = 0x20,
        WAKEUP = 0x08,
        AUTOSLEEP = 0x04,
        MEASURE1 = 0x02,
        MEASURE0 = 0x01,
    } ADXL362_POWER_CTL_reg_bits_t;

    /** SPI configuration & constructor */
    ADXL362(PinName CS = SPI_CS, PinName MOSI = SPI_MOSI, PinName MISO =
                SPI_MISO, PinName SCK = SPI_SCK);
    void frequency(int hz);

    /** Low level SPI bus comm methods */
    void reset(void);
    void write_reg(ADXL362_register_t reg, uint8_t data);
    uint8_t read_reg(ADXL362_register_t reg);
    uint16_t read_reg_u16(ADXL362_register_t reg);
    void write_reg_u16(ADXL362_register_t reg, uint16_t data);

    /** ADXL general register R/W methods */
    void set_power_ctl_reg(uint8_t data);
    void set_filter_ctl_reg(uint8_t data);
    uint8_t read_status();
    void set_mode(ADXL362_modes_t mode);

    /** ADXL X/Y/Z/T scanning methods*/
    uint64_t scan();
    uint8_t scanx_u8();
    uint16_t scanx();
    uint8_t scany_u8();
    uint16_t scany();
    uint8_t scanz_u8();
    uint16_t scanz();
    uint16_t scant();

    /** ADXL362 activity methods */
    void set_activity_threshold(uint16_t threshold);
    void set_activity_time(uint8_t time);
    void set_inactivity_threshold(uint16_t threshold);
    void set_inactivity_time(uint16_t time);
    void set_act_inact_ctl_reg(uint8_t data);

    /** ADXL362 interrupt methods */
    void set_interrupt1_pin(PinName in, uint8_t data, void (*callback_rising)(void), void (*callback_falling)(void), PinMode pull = PullNone);
    void set_interrupt2_pin(PinName in, uint8_t data, void (*callback_rising)(void), void (*callback_falling)(void), PinMode pull = PullNone);
    void enable_interrupt1();
    void enable_interrupt2();
    void disable_interrupt1();
    void disable_interrupt2();

    void set_polling_interrupt1_pin(PinName in, uint8_t data, PinMode pull = PullNone);
    void set_polling_interrupt2_pin(PinName in, uint8_t data, PinMode pull = PullNone);

    bool get_int1();
    bool get_int2();

    /** ADXL362 FIFO methods */
    uint16_t fifo_read_nr_of_entries();
    void fifo_setup(bool store_temp, ADXL362_FIFO_modes_t mode, uint16_t nr_of_entries);
    uint16_t fifo_read_u16();
    uint64_t fifo_scan();

    SPI adxl362;    ///< SPI instance of the ADXL362
    DigitalOut cs; ///< DigitalOut instance for the chipselect of the ADXL362

private:

    InterruptIn *_int1;
    InterruptIn *_int2;
    DigitalIn _int1_poll;
    DigitalIn _int2_poll;
    bool _int1_act_low;
    bool _int2_act_low;
    bool _temp_stored_in_fifo;

    const static uint8_t _DUMMY_BYTE = 0xAA;
    const static uint8_t _WRITE_REG_CMD = 0x0A; // write register
    const static uint8_t _READ_REG_CMD = 0x0B; // read register
    const static uint8_t _READ_FIFO_CMD = 0x0D; // read FIFO
    const static uint8_t _SPI_MODE = 0;
};

#endif
