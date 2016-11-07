/**
*   @file     ad7124_diag.h
*   @brief    Header file for the AD7124 wrapper used by the driver diag
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

#ifndef CN0398_DIAG_H_
#define CN0398_DIAG_H_
#include "CN0398.h"
#include "AD7124.h"

class CN0398_Diag
{
public:
    CN0398_Diag(CN0398& ad);

    void init();
    void enable_channel(int channel);
    void disable_channel(int channel);
    void enable_current_source();
    void toggle_output(int channel, uint8_t state);
    void readt();

    void readm();
    void offsetph();
    void calibp(int point);
    void readp();

    void write_reg();
    void read_reg();
    void reset();
    /* void read_data();
     void read_volt();*/
    void start_single_conversion();
    float data_to_voltage(uint32_t data);

private:
    CN0398& dut;
    const float default_offset_voltage = -0.000136;
    float offset_voltage;
    float default_calibration_ph[2][2] = {{4, 0.114451051}, {10, -0.0795903802}};
    float calibration_ph[2][2];
};


#endif /* AD5270_DIAG_H_ */
