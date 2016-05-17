/**
 *   @file     ADXL362_diag.cpp
 *   @brief    Source file for the ADXL362 wrapper used by the driver diag
 *   @author   Analog Devices Inc.
 *
 * For support please go to:
 * Github: https://github.com/analogdevicesinc/mbed-adi
 * Support: https://ez.analog.com/community/linux-device-drivers/microcontroller-no-os-drivers
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
#include <stdio.h>
#include <vector>
#include <string>
#include "ADXL362.h"
#include "ADXL362_diag.h"

extern Serial pc;
extern vector<string> cmdbuffer;
volatile bool awake;

ADXL362_Diag::ADXL362_Diag(ADXL362& ad) :
    dut(ad)
{

}

/** Low level SPI bus comm methods */
void ADXL362_Diag::reset(void)
{
    dut.reset();
    pc.printf("Reseted ADXL362");
}

void ADXL362_Diag::write_reg(void)
{
    uint8_t reg = strtol(cmdbuffer[1].c_str(), NULL, 16);
    uint8_t data = strtol(cmdbuffer[2].c_str(), NULL, 16);
    dut.write_reg(static_cast<ADXL362::ADXL362_register_t>(reg), data);
    pc.printf("Wrote reg");
}

void ADXL362_Diag::read_reg(void)
{
    uint8_t reg = strtol(cmdbuffer[1].c_str(), NULL, 16);
    uint8_t data = dut.read_reg(static_cast<ADXL362::ADXL362_register_t>(reg));
    pc.printf("Read %x ", data);
}

void ADXL362_Diag::scan(void)
{
    uint64_t data;
    data = dut.scan();
    pc.printf("Data: %x", data);
}

void ADXL362_Diag::read_status(void)
{
    uint8_t data = dut.read_status();
    pc.printf("Status - %x reg", data);
}

void ADXL362_Diag::write_ctl(void)
{
    uint8_t data = strtol(cmdbuffer[1].c_str(), NULL, 16);

    dut.set_power_ctl_reg(data);
    pc.printf("Wrote reg %x", data);
}

void ADXL362_Diag::write_ftl(void)
{
    uint8_t data = strtol(cmdbuffer[1].c_str(), NULL, 16);

    dut.set_filter_ctl_reg(data);
    pc.printf("Wrote reg %x", data);
}

void ADXL362_Diag::fifo_read_nr_of_entries(void)
{
    uint16_t data = dut.fifo_read_nr_of_entries();
    pc.printf("fifo entries - %d ", data);
}
void ADXL362_Diag::fifo_setup(void)
{
    uint8_t data = strtol(cmdbuffer[1].c_str(), NULL, 10);
    uint16_t entry = strtol(cmdbuffer[2].c_str(), NULL, 10);
    dut.fifo_setup(static_cast<bool>(data & 0x04),
                   static_cast<ADXL362::ADXL362_FIFO_modes_t>(data & 0x03), entry);
    pc.printf("Wrote reg");
}
void ADXL362_Diag::fifo_read_u16(void)
{
    uint16_t data = dut.fifo_read_u16();
    pc.printf("fifo entry - %x ", data);
}
void ADXL362_Diag::fifo_scan(void)
{
    uint64_t data = dut.fifo_scan();
    pc.printf("fifo scan - %x ", data);
}

void rising_adxl362()
{
    awake = true;
}
void falling_adxl362()
{
    awake = false;

}

