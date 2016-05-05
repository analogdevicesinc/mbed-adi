/**
*   @file     cn0216.h
*   @brief    Header file for CN0216
*   @author   Analog Devices Inc.
*
* For support please go to:
* Github: https://github.com/analogdevicesinc/mbed-adi
* Support: https://ez.analog.com/community/linux-device-drivers/microcontroller-no-os-drivers
* Product: www.analog.com/EVAL-CN0216-ARDZ
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
#include "AD7791.h"

#ifndef CN0216_H_
#define CN0216_H_

class CN0216
{
public:
    typedef enum {
        ZERO_SCALE_CALIBRATION, ///< Calibration of the zero scale value
        FULL_SCALE_CALIBRATION, ///< Calibration of the full scale value
        COMPUTE_UNITS_PER_BIT   ///< Units per LSB computation
    } CalibrationStep_t;

    CN0216(PinName CSAD7791 = D8, PinName MOSI = SPI_MOSI, PinName MISO = SPI_MISO, PinName SCK = SPI_SCK);
    void init(float cal_weight = _DEFAULT_CAL_WEIGHT, uint8_t mode_val = _DEFAULT_MODE_VAL, uint8_t filter_val = _DEFAULT_FILTER_VAL);
    void calibrate(CalibrationStep_t cal);
    float compute_weight(uint32_t data);
    uint32_t read_u32();
    float read_weight();

private:

    const static int _NUMBER_OF_SAMPLES = 20; ///< Number of samples used in calibration
    const static int _DEFAULT_MODE_VAL = AD7791::MD1 | AD7791::MD0; // POWERDOWN MODE
    const static int _DEFAULT_FILTER_VAL = AD7791::FS0 | AD7791::FS1 | AD7791::FS2;
    const static int _DEFAULT_CAL_WEIGHT = 1000.0;

    AD7791 ad7791;
    float _cal_weight;
    uint32_t _zero_scale_value;
    uint32_t _full_scale_value;
    float _weight_units_per_bit;

};

#endif
