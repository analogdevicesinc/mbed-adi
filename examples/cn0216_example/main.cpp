/**
*   @file     main.cpp
*   @brief    Main file for the CN0357-example project
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
#include "CN0216.h"

const float CAL_WEIGHT = (500.0);


Serial pc(USBTX, USBRX); ///< Serial interface to the pc

void flush_serial_buffer(void)
{
    while (pc.readable()) pc.getc();
    return;
}

void display_data(uint32_t data, float weight, uint64_t timer)
{
    pc.printf("\r\nADC Input Voltage input = %x V", data);           /* Send valid voltage input value */
    pc.printf("\r\nSensor Input Weight = %f grams", weight);         /* Send valid grams value */


    /*	pc.printf("%d ",timer);
    	pc.printf("%x ",data);
    	pc.printf("%f ",weight);*/
    pc.printf("\r\n");
}

/**
 * Project entry-point - initializes the CN0357 shield, reads the data when the ADC is ready and outputs the sensor
 * value in PPM
 */

#define SINGLE_CONVERSION
//#define CONTINOUS_CONVERSION

int main()
{
    /* Main variables */
    //t.start();
    CN0216 cn0216;
#ifdef SINGLE_CONVERSION
    cn0216.init(CAL_WEIGHT);
#elif defined CONTINOUS_CONVERSION
    cn0216.init(CAL_WEIGHT, 0x08, 0x07);
#else
#error define SINGLE_CONVERSION or CONTINOUS_CONVERSION, but not both
#endif
    /* Calibration sequence */

    pc.printf("\r\n Calibrating zero scale. Remove all weights from scale. Press any key to begin ..");
    while(!pc.readable());
    flush_serial_buffer();
    pc.printf("\r\n Calibrating . . . ");
    cn0216.calibrate(CN0216::ZERO_SCALE_CALIBRATION);
    pc.printf("done ! ");

    pc.printf("\r\n Calibrating full scale. Put calibration weight on scale. Press any key to begin ..");
    while(!pc.readable());
    flush_serial_buffer();
    pc.printf("\r\n Calibrating . . . ");
    cn0216.calibrate(CN0216::FULL_SCALE_CALIBRATION);
    pc.printf("done ! ");

    pc.printf("\r\n Calibration successful ");
    cn0216.calibrate(CN0216::COMPUTE_UNITS_PER_BIT);

    /* Infinite loop */
    uint64_t timer = 0;
    while (1) {
        wait_ms(1000);

        uint32_t data = cn0216.read_u32();
        float weight    = cn0216.compute_weight(data); //  Convert ADC data to voltage
        display_data(data, weight, timer); //  Display data thru UART

        timer++;
    }


    /* Infinite loop, never returns. */
}

