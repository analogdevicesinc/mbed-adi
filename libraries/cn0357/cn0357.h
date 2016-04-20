/**
*   @file     cn0357.h
*   @brief    Header file for CN0357
*   @author   Analog Devices Inc.
*
* For support please go to:
* Github: https://github.com/analogdevicesinc/mbed-adi
* Support: https://ez.analog.com/community/linux-device-drivers/microcontroller-no-os-drivers
* Product: www.analog.com/EVAL-CN0357-ARDZ
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

#ifndef CN0357_H
#define CN0357_H

#include "mbed.h"
#include "ad7790.h"
#include "ad5270.h"

/**
 * @brief EVAL-CN0357 toxic gas sensor shield
 */
class CN0357
{
public:

private:
    float _vref;
    float _sensor_sensitivity;
    float _sensor_range;
    float _RDACvalue;
public:
    AD7790 ad7790; ///< AD7790 instance - can be used for manual overriding
    AD5270 ad5270; ///< AD5270 instance - can be used for manual overriding

    /// CN0357 shield jumper configuration
    typedef enum {
        INTERNAL_AD7790 = 0, 	///< The shield's AD7790 is used
        EXTERNAL_ADC			///< Sensor analog output is routed to A1 pin of the shield
    } JumperConfig_t;

    CN0357(PinName CSAD7790 = D8, PinName CSAD5270 = D6, PinName MOSI = SPI_MOSI, PinName MISO = SPI_MISO, PinName SCK = SPI_SCK);
    void init(float range, float sensitivity, JumperConfig_t jp = INTERNAL_AD7790, uint8_t mode_val = _DEFAULT_MODE_VAL, uint8_t filter_val = _DEFAULT_FILTER_VAL);

    uint8_t  read_adc_status(void);
    uint16_t read_sensor(void);
    float read_sensor_voltage(void);
    float data_to_voltage(uint16_t data);
    float calc_ppm(float adcVoltage);
    float read_ppm(void);

    void  set_RDAC_value(float resistor_val);
    float get_RDAC_value(void);
    float set_sensor_parameters(float range, float sensitivity);
    float get_sensor_range(void);
    float get_sensor_sensitivity(void);

private:
    const static int _RESET = 0xff;
    const static int _DEFAULT_MODE_VAL = AD7790::MD1 | AD7790::MD0; // POWERDOWN MODE
    const static int _DEFAULT_FILTER_VAL = AD7790::FS0 | AD7790::FS1 | AD7790::FS2;
    void _rdac_init(float resistanceValue);
    void _AD7790_init(uint8_t mode_val, uint8_t filter_val);

};

#endif // CN0357_H
