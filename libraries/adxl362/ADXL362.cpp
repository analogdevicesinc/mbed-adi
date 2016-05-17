/**
*   @file     ADXL362.cpp
*   @brief    Source file for ADXL362
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

#include <stdint.h>
#include "mbed.h"
#include "ADXL362.h"

/**
 * ADXL362 constructor. Sets CS and SPI bus
 * @param CS - CS pin of the ADXL362
 * @param MOSI - MOSI pin of the ADXL362
 * @param MISO - MISO pin of the ADXL362
 * @param SCK- SCK pin of the ADXL362
 */
ADXL362::ADXL362(PinName CS, PinName MOSI, PinName MISO, PinName SCK) :
    adxl362(MOSI, MISO, SCK), cs(CS), _int1(NULL), _int2(NULL), _int1_poll(NC), _int2_poll(
        NC)
{
    cs = true; // cs is active low
    adxl362.format(8, _SPI_MODE);
    _temp_stored_in_fifo = false;
    _int1_act_low = true;
    _int2_act_low = true;
}

/**
 * Sets ADXL362 SPI bus frequency
 * @param hz - frequency in hz
 */
void ADXL362::frequency(int hz)
{
    adxl362.frequency(hz);
}

/**
 * Resets the ADXL362
 * A latency of approximately 0.5 ms is required after soft reset.
 */
void ADXL362::reset()
{
    adxl362.format(8, _SPI_MODE);
    cs = false;
    // Writing Code 0x52 (representing the letter, R, in ASCII or unicode) to this register immediately resets the ADXL362.
    write_reg(SOFT_RESET, 0x52);
    cs = true;

}

/**
 * Writes the reg register with data
 * @param reg - ADXL362_register_t register to be written
 * @param data - data to be written
 */
void ADXL362::write_reg(ADXL362_register_t reg, uint8_t data)
{
    adxl362.format(8, _SPI_MODE);
    cs = false;
    adxl362.write(_WRITE_REG_CMD);
    adxl362.write(static_cast<uint8_t>(reg));
    adxl362.write(static_cast<uint8_t>(data));
    cs = true;
}

/**
 * Reads the reg register
 * @param reg -  ADXL362_register_t register to be read
 * @return - data read from the register
 */
uint8_t ADXL362::read_reg(ADXL362_register_t reg)
{
    uint8_t ret_val;
    adxl362.format(8, _SPI_MODE);
    cs = false;
    adxl362.write(_READ_REG_CMD);
    adxl362.write(static_cast<uint8_t>(reg));
    ret_val = adxl362.write(_DUMMY_BYTE);
    cs = true;
    return ret_val;
}

/**
 * Writes 16 bit registers to the ADXL362. Performs conversion from Intel to Motorola byte order
 * @param reg -  ADXL362_register_t register to be written
 * @param data -  data to be written
 */
void ADXL362::write_reg_u16(ADXL362_register_t reg, uint16_t data)
{
    adxl362.format(8, _SPI_MODE);

    cs = false;
    adxl362.write(_WRITE_REG_CMD);
    adxl362.write(static_cast<uint8_t>(reg));
    adxl362.write(static_cast<uint8_t>(data & 0xff));
    adxl362.write(static_cast<uint8_t>((data & 0xff00) >> 8));

    cs = true;
}

/**
 * Reads 16 bit registers from the ADXL362. Performs conversion from Motorola to Intel Byte order
 * @param reg - ADXL362_register_t register to be read
 * @return - data read from the ADXL362
 */
uint16_t ADXL362::read_reg_u16(ADXL362_register_t reg)
{
    uint16_t ret_val = 0;
    adxl362.format(8, _SPI_MODE);

    cs = false;
    adxl362.write(_READ_REG_CMD);
    adxl362.write(static_cast<uint8_t>(reg));
    ret_val = adxl362.write(_DUMMY_BYTE);
    ret_val = ret_val | (adxl362.write(_DUMMY_BYTE) << 8);
    cs = true;

    return ret_val;
}

/**
 * Scans the X,Y,Z,T registers for data.
 * ADXL362 needs to be in measurement mode to read data
 * ADXL362 is set in measurement mode using ADXL362::set_mode(ADXL362::MEASUREMENT)
 * @return a 64 bit integer with the following format 0xXXYYZZTT
 */
uint64_t ADXL362::scan()
{
    uint64_t ret_val = 0;
    uint16_t x, y, z, t = 0;

    adxl362.format(8, _SPI_MODE);
    cs = false;
    adxl362.write(_READ_REG_CMD);
    adxl362.write(static_cast<uint8_t>(XDATA_L));

    x = adxl362.write(_DUMMY_BYTE);
    x = x | static_cast<uint16_t>(adxl362.write(_DUMMY_BYTE) << 8);
    y = adxl362.write(_DUMMY_BYTE);
    y = y | static_cast<uint16_t>(adxl362.write(_DUMMY_BYTE) << 8);
    z = adxl362.write(_DUMMY_BYTE);
    z = z | static_cast<uint16_t>(adxl362.write(_DUMMY_BYTE) << 8);
    t = adxl362.write(_DUMMY_BYTE);
    t = t | static_cast<uint16_t>(adxl362.write(_DUMMY_BYTE) << 8);

    ret_val = static_cast<uint64_t>(x) << 48;
    ret_val |= static_cast<uint64_t>(y) << 32;
    ret_val |= static_cast<uint64_t>(z) << 16;
    ret_val |= static_cast<uint64_t>(t);
    cs = true;
    return ret_val;
}

/**
 * Reads the X 8 bit register from the ADXL362
 * ADXL362 is set in measurement mode using ADXL362::set_mode(ADXL362::MEASUREMENT)
 * @return 8 bit X data
 */
uint8_t ADXL362::scanx_u8()
{
    return read_reg(XDATA);
}

/**
 * Reads the X 16 bit register from the ADXL362
 * ADXL362 is set in measurement mode using ADXL362::set_mode(ADXL362::MEASUREMENT)
 * @return 16 bit X data
 */
uint16_t ADXL362::scanx()
{
    return read_reg_u16(XDATA_L);
}

/**
 * Reads the Y 8 bit register from the ADXL362
 * ADXL362 is set in measurement mode using ADXL362::set_mode(ADXL362::MEASUREMENT)
 * @return 8 bit Y data
 */
uint8_t ADXL362::scany_u8()
{
    return read_reg(YDATA);
}

/**
 * Reads the Y 16 bit register from the ADXL362
 * ADXL362 is set in measurement mode using ADXL362::set_mode(ADXL362::MEASUREMENT)
 * @return 16 bit Y data
 */
uint16_t ADXL362::scany()
{
    return read_reg_u16(YDATA_L);
}

/**
 * Reads the Z 8 bit register from the ADXL362
 * ADXL362 is set in measurement mode using ADXL362::set_mode(ADXL362::MEASUREMENT)
 * @return 8 bit Z data
 */
uint8_t ADXL362::scanz_u8()
{
    return read_reg(ZDATA);
}

/**
 * Reads the Z 16 bit register from the ADXL362
 * ADXL362 is set in measurement mode using ADXL362::set_mode(ADXL362::MEASUREMENT)
 * @return 16 bit Z data
 */
uint16_t ADXL362::scanz()
{
    return read_reg_u16(ZDATA_L);
}

/**
 * Reads the T 16 bit register from the ADXL362
 * ADXL362 is set in measurement mode using ADXL362::set_mode(ADXL362::MEASUREMENT)
 * @return 16 bit T data
 */
uint16_t ADXL362::scant()
{
    return read_reg_u16(TEMP_L);
}

/**
 * Sets the STANDBY/MEASUREMENT mode of the ADXL362
 * @param mode - ADXL362_modes_t STANDBY/MEASUREMENT mode
 */
void ADXL362::set_mode(ADXL362_modes_t mode)
{
    uint8_t reg_val;
    reg_val = read_reg(POWER_CTL);
    reg_val = reg_val | static_cast<uint8_t>(mode);
    write_reg(POWER_CTL, reg_val);
}

/**
 * Sets the activity threshold registers
 * To enable activity/inactivity, the ACT_INACT_CTL reg must also be set
 * using the ADXL362::set_act_inact_ctl_reg(uint8_t data) method
 * @param threshold - activity threshold in natural format
 */
void ADXL362::set_activity_threshold(uint16_t threshold)
{
    write_reg_u16(THRESH_ACT_L, threshold);
}

/**
 * Sets the activity time register
 * To enable activity/inactivity, the ACT_INACT_CTL reg must also be set
 * using the ADXL362::set_act_inact_ctl_reg(uint8_t data) method
 * @param time - activity time
 */
void ADXL362::set_activity_time(uint8_t time)
{
    write_reg(TIME_ACT, time);
}

/**
 * Sets the inactivity threshold register
 * To enable activity/inactivity, the ACT_INACT_CTL reg must also be set
 * using the ADXL362::set_act_inact_ctl_reg(uint8_t data) method
 * @param threshold - inactivity threshold in natural format
 */
void ADXL362::set_inactivity_threshold(uint16_t threshold)
{
    write_reg_u16(THRESH_INACT_L, threshold);
}

/**
 * Sets the inactivity time register
 * To enable activity/inactivity, the ACT_INACT_CTL reg must also be set
 * using the ADXL362::set_act_inact_ctl_reg(uint8_t data) method
 * @param time - inactivity time in natural format
 */
void ADXL362::set_inactivity_time(uint16_t time)
{
    write_reg_u16(TIME_INACT_L, time);
}

/**
 * Sets the ACT_INACT_CTL register of the ADXL362
 * @param data - data to be written to the register
 */
void ADXL362::set_act_inact_ctl_reg(uint8_t data)
{
    write_reg(ACT_INACT_CTL, data);
}

/**
 * Configures INT1 output of the ADXL362 for polling use
 * @param in - uC pin connected to ADXL362's INT1
 * @param data - data to be written to INTMAP1
 * @param pull - (optional) configures pullup on In pin
 */
void ADXL362::set_polling_interrupt1_pin(PinName in, uint8_t data,
        PinMode pull)
{
    if ((data & 0x7F) != 0) {
        write_reg(INTMAP1, data);
        _int1_poll = DigitalIn(in);
        _int1_poll.mode(pull);
        if (data & 0x80) {
            _int1_act_low = true;
        } else {
            _int1_act_low = false;
        }
    }
}

/**
 * Configures INT2 output of the ADXL362 for polling use
 * @param in - uC pin connected to ADXL362's INT2
 * @param data - data to be written to INTMAP2
 * @param pull - (optional) configures pullup on In pin
 */
void ADXL362::set_polling_interrupt2_pin(PinName in, uint8_t data,
        PinMode pull)
{
    if ((data & 0x7F) != 0) {
        write_reg(INTMAP2, data);
        _int2_poll = DigitalIn(in);
        _int2_poll.mode(pull);
        if (data & 0x80) {
            _int2_act_low = true;
        } else {
            _int2_act_low = false;
        }
    }
}

/**
 * Gets the active state of the INT1 pin
 * @return true if active, false if not active
 */
bool ADXL362::get_int1()
{
    if(_int1_poll != NC) return (_int1_poll.read() != _int1_act_low);  // boolean XOR
    else return (_int1->read() != _int1_act_low);
}

/**
 * Gets the active state of the INT2 pin
 * @return true if active, false if not active
 */
bool ADXL362::get_int2()
{
    if(_int2_poll != NC) return (_int1_poll.read() != _int1_act_low); // boolean XOR
    else return (_int2->read() != _int2_act_low);
}


/**
 * Configures the INT1 pin of the ADXL362 to be used in interrupt mode
 * @param in - uC pin connected to ADXL362's INT1
 * @param data -  data to be written to INTMAP1
 * @param callback_rising - rising edge interrupt callback - can be set to NULL if no callback is required for rising edge
 * @param callback_falling - falling edge interrupt callback - can be set to NULL if no callback is required for falling edge
 * @param pull - (optional) configures pullup on In pin
 */
void ADXL362::set_interrupt1_pin(PinName in, uint8_t data,
                                 void (*callback_rising)(void), void (*callback_falling)(void), PinMode pull)
{
    if ((data & 0x7F) != 0) {
        write_reg(INTMAP1, data);
        delete _int1;
        _int1 = new InterruptIn(in);
        _int1->mode(pull);
        if(callback_falling != NULL) _int1->fall(callback_falling);
        if(callback_rising != NULL) _int1->rise(callback_rising);
        if (data & 0x80) {
            _int1_act_low = true;
        } else {
            _int1_act_low = false;
        }
    }
}


/**
 * Configures the INT2 pin of the ADXL362 to be used in interrupt mode
 * @param in - uC pin connected to ADXL362's INT2
 * @param data -  data to be written to INTMAP2
 * @param callback_rising - rising edge interrupt callback - can be set to NULL if no callback is required for rising edge
 * @param callback_falling - falling edge interrupt callback - can be set to NULL if no callback is required for falling edge
 * @param pull - (optional) configures pullup on In pin
 */
void ADXL362::set_interrupt2_pin(PinName in, uint8_t data,
                                 void (*callback_rising)(void), void (*callback_falling)(void), PinMode pull)
{
    if ((data & 0x7F) != 0) {
        write_reg(INTMAP2, data);
        delete _int2;
        _int2 = new InterruptIn(in);
        _int2->mode(pull);
        if(callback_falling != NULL) _int2->fall(callback_falling);
        if(callback_rising != NULL) _int2->rise(callback_rising);
        if (data & 0x80) {
            _int2_act_low = true;
        } else {
            _int2_act_low = false;
        }
    }
}

/**
 * Enables external interrupt registration for pin configured as INT1
 * To enable this interrupt, it must first be configured using ADXL362::set_interrupt1_pin()
 */
void ADXL362::enable_interrupt1()
{
    _int1->enable_irq();
}

/**
 * Enables external interrupt registration  for pin configured as INT2
 * * To enable this interrupt, it must first be configured using ADXL362::set_interrupt2_pin()
 */
void ADXL362::enable_interrupt2()
{
    _int2->enable_irq();
}

/**
 * Disables external interrupt registration  for pin configured as INT1
 */
void ADXL362::disable_interrupt1()
{
    _int1->disable_irq();
}

/**
 * Disables external interrupt registration  for pin configured as INT2
 */
void ADXL362::disable_interrupt2()
{
    _int2->disable_irq();
}

/**
 * Sets the POWER_CTL register
 * @param data - data to be written to the register
 */
void ADXL362::set_power_ctl_reg(uint8_t data)
{
    write_reg(POWER_CTL, data);
}

/**
 * Sets the FILTER_CTL register
 * @param data - data to be written to the register
 */
void ADXL362::set_filter_ctl_reg(uint8_t data)
{
    write_reg(FILTER_CTL, data);
}

/**
 * Reads the STATUS register of the ADXL362
 * @return - data in the status register
 */
uint8_t ADXL362::read_status()
{
    return read_reg(STATUS);
}

/**
 * Reads the FIFO_ENTRIES_L and FIFO_ENTRIES_H register
 * @return the number of entries in the FIFO
 */
uint16_t ADXL362::fifo_read_nr_of_entries()
{
    return read_reg_u16(FIFO_ENTRIES_L);
}

/**
 * Setup for the FIFO
 * @param store_temp - boolean, true - temperature will be stored in the fifo. false otherwise
 * @param mode - ADXL362_FIFO_modes_t fifo mode
 * @param nr_of_entries - number of entries in the FIFO
 */
void ADXL362::fifo_setup(bool store_temp, ADXL362_FIFO_modes_t mode, uint16_t nr_of_entries)
{
    uint8_t fifo_ctl = static_cast<uint8_t>(mode);
    _temp_stored_in_fifo = store_temp;

    fifo_ctl = fifo_ctl | (static_cast<uint8_t>(_temp_stored_in_fifo) << 2);

    if (nr_of_entries > 0xff) {
        fifo_ctl = fifo_ctl | static_cast<uint8_t>(AH);
    }
    write_reg(FIFO_CONTROL, fifo_ctl);
    write_reg(FIFO_SAMPLES, static_cast<uint8_t>(nr_of_entries & 0xff));

}

/**
 * Reads a FIFO entry
 * @return FIFO entry
 */
uint16_t ADXL362::fifo_read_u16()
{
    uint16_t ret_val = 0;
    adxl362.format(8, _SPI_MODE);

    cs = false;
    adxl362.write(_READ_FIFO_CMD);
    ret_val = adxl362.write(_DUMMY_BYTE);
    ret_val = (ret_val) | static_cast<uint16_t>(adxl362.write(_DUMMY_BYTE) << 8);
    cs = true;

    return ret_val;
}

/**
 * Reads 3(4) bytes from the FIFO(if store_temp was set), assembles the data in the format used by the scan method
 * ADXL362::fifo_setup() needs to be called before calling fifo_scan to ensure correct fifo operation
 * fifo_scan and fifo_read_u16 should not be used as fifo_read_u16 disaligns the fifo therefore
 * fifo_scan will return data from multiple samples
 * @return scanned data from the fifo in the 0xXXYYZZTT format
 */
uint64_t ADXL362::fifo_scan()
{

    uint64_t ret_val = 0;
    uint16_t x = 0, y = 0, z = 0, dummy, t = 0, sample_type;

    adxl362.format(8, _SPI_MODE);
    cs = false;
    adxl362.write(_READ_FIFO_CMD);
    uint8_t samples = (_temp_stored_in_fifo) ? 4 : 3;
    for(uint8_t i = 0; i < samples; i++) {
        dummy = adxl362.write(_DUMMY_BYTE);
        dummy = dummy | static_cast<uint16_t>(adxl362.write(_DUMMY_BYTE) << 8);
        sample_type = (dummy & 0xc000) >> 14;
        dummy = dummy & 0x3fff;
        switch(sample_type) {
            case 0: // x
                x = dummy;
                break;
            case 1: // y
                y = dummy;
                break;
            case 2: // z
                z = dummy;
                break;
            case 3: // temp
                t = dummy;
                break;
        }

    }

    // format xxyyzztt
    ret_val = static_cast<uint64_t> (x) << 48;
    ret_val |= static_cast<uint64_t>(y) << 32;
    ret_val |= static_cast<uint64_t>(z) << 16;
    ret_val |= static_cast<uint64_t>(t);
    cs = true;
    return ret_val;
}
