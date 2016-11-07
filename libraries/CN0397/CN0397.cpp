/**
*   @file     CN0397.cpp
*   @brief    Source file for the CN0397
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

#include "CN0397.h"
#include "AD7798.h"
#include "Timer.h"
#include <stdio.h>
#include <math.h>
#include <mbed.h>

extern Serial pc;


void flush_serial()
{
    wait_ms(10); // wait for all data to come through
    while (pc.readable())  pc.getc();
}


CN0397::CN0397(PinName cs) : ad7798(cs)
{

}
void CN0397::display_data(void)
{

    uint8_t channel = 0, i;

    /* for(channel = 0; channel < CHANNELS; channel++){

        pc.printf("%s (channel %d):\n", colour[channel], (channel + 1));
        pc.printf("\tADC output = %u (%#06x)\n", adcValue[channel], adcValue[channel]);
        pc.printf("\tVoltage = %f mV\n", voltageValue[channel]);
        pc.printf("\tLight Intensity = %.2f lux\n", intensityValue[channel]);
        pc.printf("\tLight Concentration = %.2f%c\n", lightConcentration[channel], 37);

     }*/


    for(channel = 0; channel < CHANNELS; channel++) {

        pc.printf("\t\t\033[2;%dm%s\033[0m channel:\t\t", ColorPrint[channel], colour[channel]);
    }

    pc.printf("\n");
    pc.printf("\t");

    /*
    	for(channel = 0; channel < CHANNELS; channel++) {

    		pc.printf("\t\t\tADC output = %u (%#06x)", adcValue[channel], adcValue[channel]);
    	}

    	pc.printf("\n");
    	pc.printf("\t");

    */
    for(channel = 0; channel < CHANNELS; channel++) {

        pc.printf("\t\tLight Intensity = %.2f lux", intensityValue[channel]);
    }

    pc.printf("\n");
    pc.printf("\t");


    for(channel = 0; channel < CHANNELS; channel++) {

        pc.printf("\t\tLight Concentration = %.2f%c", lightConcentration[channel], 37);
    }

    pc.printf("\n");/*pc.printf("\t");

	for(channel = 0; channel < CHANNELS; channel++) {

		pc.printf("\t\t\tLight Concentration = %.2f%c", lightConcentration[channel], 37);
	}

	pc.printf("\n");pc.printf("\t");

	/* for(channel = 0; channel < CHANNELS; channel++){

             pc.printf("\t\t\t\033[2;%dm|\033[0m\t", ColorPrint[channel]);

     }*/

    for(channel = 0; channel < 5; channel++) {
        pc.printf("\n");
    }

    //  pc.printf("\n");
    //  pc.printf("\n");
    pc.printf("\n");
}


void CN0397::data_to_voltage(uint16_t adcValue, float *voltage)
{

    *voltage = (float)(adcValue * V_REF) / (float)(_2_16 * gainAdc);

}

void CN0397::init(void)
{
    ad7798.reset();
    uint8_t channel;

    if(ad7798.init()) {

        ad7798.set_coding_mode(AD7798_UNIPOLAR);
        ad7798.set_mode(AD7798_MODE_SINGLE);
        ad7798.set_gain(ADC_GAIN);
        ad7798.set_filter(ADC_SPS);
        ad7798.set_reference(AD7798_REFDET_ENA);

    }

    gainAdc = Gain[ADC_GAIN];
#ifdef USE_CALIBRATION
    pc.printf("Calibrate the system:\n");
    pc.printf("\n");

    for(channel = 0; channel < CHANNELS; channel++) {

        pc.printf("\tCalibrate channel %d: be sure that %s photodiode is cover and press <ENTER>.\n", (Channels[channel] + 1), colour[channel]);
        pc.getc();
        flush_serial();
        calibration(Channels[channel]);
        pc.printf("\t\tChannel %d is calibrated!\n", (Channels[channel] + 1));
        pc.printf("\n");

    }

    printf("System calibration complete!\n");
    printf("\n");
    printf("\n");

#endif
}

void CN0397::calc_light_intensity(uint8_t channel, uint16_t adcValue, float *intensity)
{

    *intensity = adcValue * Lux_LSB[channel];

}

void CN0397::calc_light_concentration(uint8_t channel, float intensity, float *conc)
{

    *conc = (intensity * 100) / Optimal_Levels[channel];

}

void CN0397::set_app_data(void)
{
    uint8_t channel, rgbChannel;
    uint16_t *adcData = &adcValue[0];
    float *voltageData = &voltageValue[0], *intensityData = &intensityValue[0];
    float *concData = &lightConcentration[0];

    for(channel = 0; channel < CHANNELS; channel++) {

        rgbChannel = Channels[channel];

        ad7798.set_channel(channel);

        adcData = &adcValue[rgbChannel];
        voltageData = &voltageValue[rgbChannel];
        intensityData = &intensityValue[rgbChannel];
        concData = &lightConcentration[rgbChannel];
        ad7798.read_data(channel, adcData);
        data_to_voltage(*adcData, voltageData);
        calc_light_intensity(rgbChannel, *adcData, intensityData);
        calc_light_concentration(rgbChannel, *intensityData, concData);

        /*    adcData++;
            voltageData++;
            intensityData++;
            concData++;*/

    }
}

void CN0397::calibration(uint8_t channel)
{

    uint16_t setValue;

    ad7798.set_channel(channel);  //select channel to calibrate

    // Perform system zero-scale calibration
    setValue = ad7798.get_register_value(AD7798_REG_MODE, 2);
    setValue &= ~(AD7798_MODE_SEL(0x07));
    setValue |= AD7798_MODE_SEL(AD7798_MODE_CAL_SYS_ZERO);
    ad7798.set_register_value(AD7798_REG_MODE, setValue, 2);

    while((ad7798.get_register_value( AD7798_REG_STAT, 1) & channel) != channel); // wait for RDY bit to go low

    while(ad7798.get_register_value(AD7798_REG_MODE, 2) != 0x4005);    // wait for ADC to go in idle mode


}




