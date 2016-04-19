/**
*   @file     cn0357.cpp
*   @brief    Source file for CN0357
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

#include "mbed.h"
#include "AD7790.h"
#include "AD5270.h"
#include "cn0357.h"

/**
 * @brief CN0357 constructor
 * @param CSAD7790 - (optional)chip select of the AD7790
 * @param CSAD5270 - (optional)chip select of the AD5270
 * @param MOSI - (optional)pin of the SPI interface
 * @param MISO - (optional)pin of the SPI interface
 * @param SCK  - (optional)pin of the SPI interface
 */
CN0357::CN0357(PinName CSAD7790, PinName CSAD5270, PinName MOSI, PinName MISO,
               PinName SCK) :
    _vref(1.2), _sensor_sensitivity(0), _sensor_range(0), _RDACvalue(0),
    ad7790(_vref, CSAD7790, MOSI, MISO, SCK), ad5270(CSAD5270, 20000.0, MOSI, MISO, SCK)

{
}

/**
 * @brief initializes the AD7790 and the AD5270
 * @param range - range of the sensor used (in ppm)
 * @param sensitivity - sensitivity of the sensor (A/ppm)
 * @param jp - (optional)jumper configuration of the CN0357
 * @param mode_val - (optional)if jp is set to InternalADC, configures the mode register of the Internal ADC
 * @param filter_val - (optional)if jp is set to InternalADC, configures the filter register of the Internal ADC
 */
void CN0357::init(float range, float sensitivity, JumperConfig_t jp, uint8_t mode_val, uint8_t filter_val)
{
    ad5270.frequency(500000);
    ad7790.frequency(500000);

    float resistance = set_sensor_parameters(range, sensitivity);

    if(jp == INTERNAL_AD7790) {
        _AD7790_init(mode_val, filter_val);
    }
    _rdac_init(resistance);
}

/**
 * @brief initializes the RDAC and sets SDO to HiZ
 * @param resistance - resistance value to initialize the RDAC
 * @return
 */
void CN0357::_rdac_init(float resistance)
{
    /* RDAC initialization*/
    /* Compute for the nearest RDAC value from given resistance and save data to the structure */
    set_RDAC_value(resistance);
    /* Set AD5270 SDO to Hi-Z */
    ad5270.set_SDO_HiZ();
}

/**
 * @brief initializes the AD7790
 * @param mode_val -  configures the mode register of the Internal ADC
 * @param filter_val -  configures the filter register of the Internal ADC
 */
void CN0357::_AD7790_init(uint8_t mode_val, uint8_t filter_val)
{
    ad7790.reset();
    wait_ms(50);

    ad7790.write_mode_reg(mode_val);
    wait_us(2);

    ad7790.write_filter_reg(filter_val);
    wait_ms(50);
}

/**
 * @brief reads the status register of the AD7790
 * @return status register value
 */
uint8_t CN0357::read_adc_status(void)
{
    return ad7790.read_status_reg();
}

/**
 * @brief reads the ADC and computes the sensor voltage
 * @return sensor voltage
 */
float CN0357::read_sensor_voltage(void)
{
    return ad7790.read_voltage();
}

/**
 * @brief reads the data register of the AD7790
 * @return data register value
 */
uint16_t CN0357::read_sensor(void)
{
    return ad7790.read_u16();
}

/**
 * @brief reads and computes the sensor reading in PPM
 * @return value of the sensor reading in PPM
 */
float CN0357::read_ppm()
{
    return calc_ppm(ad7790.read_voltage()); /* Convert voltage to Gas concentration*/
}

/**
 * @brief computes a value in PPM from a reading received as a param
 * @param adcVoltage - voltage to be converted to PPM
 * @return sensor value in PPM
 */
float CN0357::calc_ppm(float adcVoltage)
{
    float fConcentration = 0;
    fConcentration = (fabs(adcVoltage) / _RDACvalue) / _sensor_sensitivity;
    return fConcentration;
}

/**
 * @brief computes voltage from a 16 bit ADC value received as a parameter
 * @param data - ADC value
 * @return sensor voltage
 */
float CN0357::data_to_voltage(uint16_t data)
{
    return ad7790.data_to_voltage(data);
}

/**
 * @brief sets a new value for the RDAC
 * @param resistance new value for the resistance
 * @return none
 */
void CN0357::set_RDAC_value(float resistance)
{
    _RDACvalue = ad5270.write_RDAC(resistance);
}

/**
 * @brief getter method for RDAC value
 * @return value of the RDAC in ohms
 */
float CN0357::get_RDAC_value()
{
    return _RDACvalue;
}

/**
 * @brief set sensor range and sensitivity
 * sets sensor range, sensitivity
 * returns suggested resistance value for feedback resistor
 * @param range - range of the sensor used (in ppm)
 * @param sensitivity - sensitivity of the sensor (in A/ppm)
 * @return suggested resistance value for feedback resistor
 */
float CN0357::set_sensor_parameters(float range, float sensitivity)
{
    _sensor_sensitivity = static_cast<float>(sensitivity);
    _sensor_range = range;
    return (_vref / (static_cast<float>(_sensor_range * _sensor_sensitivity)));
}

/**
 * @brief getter method for sensor sensitivity
 * @return sensor sensitivity (in A/ppm)
 */
float CN0357::get_sensor_sensitivity()
{
    return _sensor_sensitivity;
}

/**
 * @brief getter method for sensor range
 * @return sensor range (in ppm)
 */
float CN0357::get_sensor_range()
{
    return _sensor_range;
}
