/**
*   @file     ad5270_diag.cpp
*   @brief    Source file for the AD5270 wrapper used by the driver diag
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
#include "AD5270_Diag.h"

extern Serial pc;
extern vector<string> cmdbuffer;

AD5270_Diag::AD5270_Diag(AD5270& ad) : dut(ad)
{

}
void AD5270_Diag::enable_50TP_programming()
{
    dut.enable_50TP_programming();
    pc.printf("Enabled 50TP prog");

}
void AD5270_Diag::store_50TP()
{
    dut.store_50TP();
    pc.printf("50TP stored");
}
void AD5270_Diag::disable_50TP_programming()
{
    dut.disable_50TP_programming();
    pc.printf("Disabled 50TP prog");
}

void AD5270_Diag::write_RDAC()
{
    float res = strtof(cmdbuffer[1].c_str(), NULL);
    dut.write_RDAC(res);
    pc.printf("Wrote %f", res);
}
void AD5270_Diag::read_RDAC()
{
    pc.printf("Read %f", dut.read_RDAC());
}
void AD5270_Diag::write_cmd()
{
    uint8_t reg = strtol(cmdbuffer[1].c_str(), NULL, 16);
    uint8_t regVal = strtol(cmdbuffer[2].c_str(), NULL, 16);
    pc.printf("Returned %x: ", dut.write_cmd(reg, regVal));
}

void AD5270_Diag::set_HiZ()
{
    pc.printf("SDO set to HiZ");
    dut.set_SDO_HiZ();
}

void AD5270_Diag::read_50TP_last_address(void)
{
    pc.printf("Returned %x:", dut.read_50TP_last_address());
}
void AD5270_Diag::read_50TP_memory(void)
{
    uint8_t reg = strtol(cmdbuffer[1].c_str(), NULL, 16);
    pc.printf("Returned %x", dut.read_50TP_memory(reg));
}

void AD5270_Diag::write_ctrl_reg(void)
{
    uint8_t val = strtol(cmdbuffer[1].c_str(), NULL, 16);
    dut.write_ctrl_reg(val);
    pc.printf("Wrote %x to ctrl_reg", val );
}
void AD5270_Diag::read_ctrl_reg(void)
{
    pc.printf("Read %x from ctrl_reg", dut.read_ctrl_reg());
}

void AD5270_Diag::reset_RDAC(void)
{
    dut.reset_RDAC();
    pc.printf("Resetted rdac");
}
void AD5270_Diag::change_mode(void)
{
    uint8_t val = strtol(cmdbuffer[1].c_str(), NULL, 16);
    dut.change_mode(static_cast<AD5270::AD5270Modes_t>(val));
    pc.printf("Changed mode to %x", val);
}

void AD5270_Diag::write_wiper_reg(void)
{
    uint16_t val = strtol(cmdbuffer[1].c_str(), NULL, 16);
    dut.write_wiper_reg(val);
    pc.printf("Wrote %x to wiper", val);
}
void AD5270_Diag::read_wiper_reg(void)
{
    pc.printf("Read %x from wiper", dut.read_wiper_reg());
}
