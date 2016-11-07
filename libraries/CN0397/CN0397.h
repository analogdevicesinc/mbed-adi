/**
*   @file     CN0397.cpp
*   @brief    Header file for the CN0397
*   @author   Analog Devices Inc.
*
* For support please go to:
* Github: https://github.com/analogdevicesinc/mbed-adi
* Support: https://ez.analog.com/community/linux-device-drivers/microcontroller-no-os-drivers
* Product: www.analog.com/EVAL-CN0397-ARDZ
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

#ifndef CN0397_H_
#define CN0397_H_

#include <stdio.h>
#include <string.h>
#include "AD7798.h"

#define REGISTERS_VALUES     3
#define CONVERSION_DATA      4

#define V_REF                3150.0    // [mV]
#define _2_16                65535.0   // 2^16


/* Available settings:
 * AD7798_CH_AIN1P_AIN1M - select channel 1
 * AD7798_CH_AIN2P_AIN2M - select channel 2
 * AD7798_CH_AIN3P_AIN3M - select channel 3
 */
#define ADC_CHANNEL   AD7798_CH_AIN2P_AIN2M

/* Available settings:
 *  AD7798_GAIN_1, AD7798_GAIN_2,
 *  AD7798_GAIN_4, AD7798_GAIN_8,
 *  AD7798_GAIN_16, AD7798_GAIN_32,
 *  AD7798_GAIN_64, AD7798_GAIN_128
 */
#define ADC_GAIN      AD7798_GAIN_1
/* Available settings:
 * Check available value from datasheet
 */
#define ADC_SPS        0x05  /*50SPS*/


/* Available settings:
 * How often to display output values on terminal -> msec
 */
#define DISPLAY_REFRESH        1000   /*[msec]*/

#define CHANNELS  3


#define USE_CALIBRATION      /* Select if you want to use system zero-scale calibration before reading the system data*/
/**
 * Helper function used to flush the serial interface
 */
void flush_serial();

/**
 * @brief The CN0397 shield class
 */
class CN0397
{
public:
	/**
	 * @brief The CN0397 class constructor
	 */
    CN0397(PinName cs);
    /**
     * @brief Initialization method of the class. Initializes the AD7798 and initiates calibration if needed
     */
    void init(void);

    /**
     * @brief Displays data on the serial interface
     */
    void display_data(void);

    /**
     * @brief Converts ADC counts to voltage
     * @param adcValue - ADC counts
     * @param voltage - computed voltage
     */
    void data_to_voltage(uint16_t adcValue, float *voltage);

    /**
     * @brief Computes light intensity of the channel
     * @param channel - channel to be converted
     * @param adcValue - ADC counts
     * @param intensity - computed light intensity
     */
    void calc_light_intensity(uint8_t channel, uint16_t adcValue, float *intensity);

    /**
     * @brief Computes light concentration from light intensity
     * @param channel - channel to be converted
     * @param intensity - light intensity
     * @param conc - computed light concentration
     */
    void calc_light_concentration(uint8_t channel, float intensity, float *conc);

    /**
     * @brief Reads the ADC channels and computes intensity and concentration
     */
    void set_app_data(void);

    /**
     * @brief Calibrates the channel
     * @param channel - channel to be calibrated
     */
    void calibration(uint8_t channel);

    /**
     * @brief Instance of the AD7798
     */
    AD7798 ad7798;
private:

    uint8_t statusReg, idReg, ioReg, gainAdc;
    uint16_t modeReg, configReg, offsetReg, fullscaleReg, dataReg;
    uint16_t adcValue[3];
    float voltageValue[3], intensityValue[3], lightConcentration[3];
    const uint8_t Channels[3] = { 1, 0, 2};
    const char colour[3][6] = {
        "RED", "GREEN", "BLUE",
    };

    const uint8_t ColorPrint[3] = { 31, 32, 34 };
    const uint8_t Gain[8] = { 1, 2, 4, 8, 16, 32, 64, 128};
    const float Lux_LSB[3] = {2.122, 2.124, 2.113};
    const float Optimal_Levels[3] = {26909.0, 8880.0, 26909.0};

};

#endif /* CN0397_H_ */

