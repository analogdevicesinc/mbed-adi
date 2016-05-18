/**
*   @file     ad7790_diag.cpp
*   @brief    Source file for the AD7790 wrapper used by the driver diag
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
#include "AD7790_Diag.h"

extern Serial pc;
extern vector<string> cmdbuffer;

AD7790_Diag::AD7790_Diag(AD7790& ad) : dut(ad)
{

}

void AD7790_Diag::init()
{

}
void AD7790_Diag::reset()
{
    dut.reset();
    pc.printf("Reseted AD7790");
}

void AD7790_Diag::write_mode()
{
    uint8_t regVal = strtol(cmdbuffer[1].c_str(), NULL, 16);
    dut.write_mode_reg(regVal);
    pc.printf("Wrote mode");
}
void AD7790_Diag::read_mode()
{
    pc.printf("Mode reg: %x ", dut.read_mode_reg());
}

void AD7790_Diag::write_filter()
{
    uint8_t regVal = strtol(cmdbuffer[1].c_str(), NULL, 16);
    dut.write_filter_reg(regVal);
    pc.printf("Wrote filter");
}
void AD7790_Diag::read_filter()
{
    pc.printf("Returned: %x ", dut.read_filter_reg());
}
void AD7790_Diag::read_data()
{
    pc.printf("Data reg: %x ", dut.read_data_reg());
}

void AD7790_Diag::read_status()
{
    pc.printf("Status reg: %x ", dut.read_status_reg());
}

void AD7790_Diag::read_u16()
{
    pc.printf("Data reg: %x ", dut.read_u16());
}
void AD7790_Diag::read_voltage()
{
    pc.printf("Voltage: %f ", dut.read_voltage());
}
void AD7790_Diag::set_continous_mode()
{
    uint8_t regVal = strtol(cmdbuffer[1].c_str(), NULL, 16);
    dut.set_conversion_mode(static_cast<AD7790::AD7790Mode_t>(regVal));
    pc.printf("Mode set to %d", regVal);
}
void AD7790_Diag::set_reference_voltage()
{
    float ref = strtof(cmdbuffer[1].c_str(), NULL);
    dut.set_reference_voltage(ref);
    pc.printf("Reference Voltage set to %f", ref);
}

void AD7790_Diag::set_channel()
{
    uint8_t regVal = strtol(cmdbuffer[1].c_str(), NULL, 16);
    dut.set_channel(static_cast<AD7790::AD7790Channel_t>(regVal));
    pc.printf("Mode set to %d", regVal);
}
