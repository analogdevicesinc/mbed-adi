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
#include "Thermocouple.h"
#include "CN0391.h"

Serial pc(SERIAL_TX, SERIAL_RX);

int main()
{
    pc.baud(115200);
    //Thermocouple *tcarray[4] = {new Thermocouple_Type_E, new Thermocouple_Type_E,new Thermocouple_Type_B,new Thermocouple_Type_B}
    CN0391 cn0391(D10);
    pc.printf("Reset the AD7124\r\n");
    cn0391.reset();
    wait_ms(500);
    pc.printf("Init the AD7124\r\n");
    cn0391.init();
    wait_ms(500);
    pc.printf("Done\r\n");

    cn0391.set_thermocouple_type(CN0391::CHANNEL_P1, new Thermocouple_Type_E);
    cn0391.set_thermocouple_type(CN0391::CHANNEL_P2, new Thermocouple_Type_B);
    cn0391.set_thermocouple_type(CN0391::CHANNEL_P3, new Thermocouple_Type_J);
    cn0391.set_thermocouple_type(CN0391::CHANNEL_P4, new Thermocouple_Type_S);



    pc.printf("Calibration current for channel 1: %f\r\n", cn0391.calibrate(CN0391::CHANNEL_P1));
    pc.printf("Calibration current for channel 2: %f\r\n", cn0391.calibrate(CN0391::CHANNEL_P2));
    pc.printf("Calibration current for channel 3: %f\r\n", cn0391.calibrate(CN0391::CHANNEL_P3));
    pc.printf("Calibration current for channel 4: %f\r\n", cn0391.calibrate(CN0391::CHANNEL_P4));
    while(1) {
        pc.printf("Thermocouple temperature channel 1: %f\r\n", cn0391.read_channel(CN0391::CHANNEL_P1));
        pc.printf("Thermocouple temperature channel 2: %f\r\n", cn0391.read_channel(CN0391::CHANNEL_P2));
        pc.printf("Thermocouple temperature channel 3: %f\r\n", cn0391.read_channel(CN0391::CHANNEL_P3));
        pc.printf("Thermocouple temperature channel 4: %f\r\n", cn0391.read_channel(CN0391::CHANNEL_P4));

        pc.printf("\r\n");

        wait_ms(1000);
    }
    return 0;
}
