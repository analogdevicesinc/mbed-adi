/**
 *   @file     ad7124_diag.cpp
 *   @brief    Source file for the AD7124 wrapper used by the driver diag
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
#include "CN0398_Diag.h"

#include "Thermocouple.h"


extern Serial pc;
extern vector<string> cmdbuffer;

//#define CALIBRATION
#define VH400
//#define EC5


CN0398_Diag::CN0398_Diag(CN0398& ad) :
    dut(ad), offset_voltage(default_offset_voltage)
{
    calibration_ph[0][0] = default_calibration_ph[0][0];
    calibration_ph[0][1] = default_calibration_ph[0][1];
    calibration_ph[1][0] = default_calibration_ph[1][0];
    calibration_ph[1][1] = default_calibration_ph[1][1];
}

void CN0398_Diag::init()
{
    dut.reset();
    dut.setup();
    dut.init();
}


void CN0398_Diag::write_reg()
{
    uint8_t reg = strtol(cmdbuffer[1].c_str(), NULL, 16);
    uint32_t regData = strtol(cmdbuffer[2].c_str(), NULL, 16);
    //dut.write_reg(reg, regData);
    dut.ad7124.WriteDeviceRegister(static_cast<AD7124::ad7124_registers>(reg),
                                   regData);
    pc.printf("Wrote mode");
}
void CN0398_Diag::read_reg()
{
    uint8_t regVal = strtol(cmdbuffer[1].c_str(), NULL, 16);
    pc.printf("Mode reg: %x ",
              dut.ad7124.ReadDeviceRegister(
                  static_cast<AD7124::ad7124_registers>(regVal)));
}
void CN0398_Diag::reset()
{
    dut.ad7124.frequency(500000);
    dut.ad7124.Reset();
    pc.printf("Reseted AD7124");
    wait_ms(500);
}

float CN0398_Diag::data_to_voltage(uint32_t data)
{
    data = data & 0xFFFFFF;
    return ((data / static_cast<float>(0xFFFFFF / 2)) - 1) * (2.5 / 1);
}

void CN0398_Diag::enable_channel(int channel)
{
    dut.enable_channel(channel);
}

void CN0398_Diag::disable_channel(int channel)
{
    dut.disable_channel(channel);
}

void CN0398_Diag::enable_current_source()
{
    dut.enable_current_source0(11);
    dut.enable_current_source1(12);
    pc.printf("Enabled 500uA current sources on channel 11, 12\r\n");
}

void CN0398_Diag::toggle_output(int channel, uint8_t state)
{
    dut.set_digital_output(static_cast<CN0398::ad_digital_output_t>(channel), state);
}


void CN0398_Diag::readt()
{

    //enable_current_source();
    enable_channel(2);
    //wait_ms(100);
    start_single_conversion();
    //wait_ms(100);
    if (dut.ad7124.WaitForConvReady(10000) == -3) {
        pc.printf("TIMEOUT");
        return;
    }
    int32_t data;
    dut.ad7124.ReadData(&data);
    disable_channel(2);

    pc.printf("Channel: %d\r\n", data & 0xff);
    pc.printf("Data reg: %x \r\n", data);
    float volt = dut.data_to_voltage(data >> 8, 16);
    pc.printf("Voltage = %f\r\n", volt);
    data = (data >> 8) & 0x00ffffff;
    float a1 = (static_cast<float>(data) - (1 << 23));
    float a2 = a1 * 5000;
    float a3 = 16.0 * (1 << 23);

    float resistance = a2 / a3; //((data- (1<<23)) * 5000.0) / 16.0*(1<<23) ;
    pc.printf("Resistance: %f\r\n", resistance);
    pc.printf("Temperature = %f\r\n", (resistance - 100.0) / 0.385);

}

void CN0398_Diag::readm()
{

    toggle_output(0, 1);
#ifdef EC5
    wait_ms(10);
#endif
    enable_channel(1);
    //wait_ms(100);
    start_single_conversion();
    //wait_ms(100);
    if (dut.ad7124.WaitForConvReady(10000) == -3) {
        pc.printf("TIMEOUT");
        return;
    }
    int32_t data;
    dut.ad7124.ReadData(&data);
    disable_channel(1);
    toggle_output(0, 0);

    pc.printf("Channel: %d\r\n", data & 0xff);
    pc.printf("Data reg: %x \r\n", data);
    float volt = dut.data_to_voltage(data >> 8, 16);
    pc.printf("Voltage = %f\r\n", volt);
    data = (data >> 8) & 0x00ffffff;


    float moisture;
#ifdef VH400
    moisture = -1.18467 + 21.5371 * volt - 110.996 * (pow(volt, 2)) + 397.025 * (pow(volt, 3)) - 666.986 * (pow(volt, 4)) + 569.236 * (pow(volt, 5)) - 246.005 * (pow(volt, 6)) + 49.4867 * (pow(volt, 7)) - 3.37077 * (pow(volt, 8));
#elif EC5
    moisture = 0.000992 * (volt * 1000) - 0.45;
#endif
    pc.printf("Moisture = %f\r\n", moisture);



}

void CN0398_Diag::offsetph()
{
    enable_channel(0);
    //wait_ms(100);
    start_single_conversion();
    //wait_ms(100);
    if (dut.ad7124.WaitForConvReady(10000) == -3) {
        pc.printf("TIMEOUT");
        return;
    }
    int32_t data;
    dut.ad7124.ReadData(&data);
    disable_channel(0);
    pc.printf("Channel: %d\r\n", data & 0xff);
    pc.printf("Data reg: %x \r\n", data);
    float volt = dut.data_to_voltage(data >> 8, 1);
    pc.printf("Voltage = %f\r\n", volt);
    offset_voltage = volt;

}

void CN0398_Diag::calibp(int point)
{
    calibration_ph[point][0] =  strtof(cmdbuffer[1].c_str(), NULL);
    enable_channel(0);
    start_single_conversion();
    if (dut.ad7124.WaitForConvReady(10000) == -3) {
        pc.printf("TIMEOUT");
        return;
    }
    int32_t data;
    dut.ad7124.ReadData(&data);
    disable_channel(0);
    pc.printf("Channel: %d\r\n", data & 0xff);
    pc.printf("Data reg: %x \r\n", data);
    float volt = dut.data_to_voltage(data >> 8, 1);
    pc.printf("Voltage = %f\r\n", volt);
    calibration_ph[point][1] =  volt;

}
void CN0398_Diag::readp()
{
    enable_channel(0);
    start_single_conversion();
    if (dut.ad7124.WaitForConvReady(10000) == -3) {
        pc.printf("TIMEOUT");
        return;
    }
    int32_t data;
    dut.ad7124.ReadData(&data);
    disable_channel(0);
    pc.printf("Channel: %d\r\n", data & 0xff);
    pc.printf("Data reg: %x \r\n", data);
    float volt = dut.data_to_voltage(data >> 8, 1);
    pc.printf("Voltage = %f\r\n", volt);
    float m =  (calibration_ph[1][0] - calibration_ph[0][0]) / (calibration_ph[1][1] - calibration_ph[0][1]);
    pc.printf("pH = %f", m * (volt - calibration_ph[1][1]) + calibration_ph[1][0]);

}


void CN0398_Diag::start_single_conversion()
{
    dut.start_single_conversion();
}

/*void CN0398_Diag::read_data()
{
    int32_t data;
    float volt;
    float cal_current = 0;

    for(int i = 0; i < 8; i++) {
        if(i % 2 == 0) {
            //enable_current_source(i+1);
        } else {
            enable_current_source(i);
            enable_channel(8); // calibration channel
            start_single_conversion();
            if (dut.WaitForConvReady(10000) == -3) {
                pc.printf("TIMEOUT");
                return;
            }

            dut.ReadData(&data);
            disable_channel(8);

            pc.printf("Channel: %d\r\n", data & 0xff);
            pc.printf("Data reg: %x \r\n", data);
            volt = data_to_voltage(data >> 8);
            pc.printf("Voltage = %f\r\n", volt);
            cal_current = conversion_fkt[8](volt, 0);
            conversion_results[8] = cal_current;

        }

        enable_channel(i);
        start_single_conversion();


        if (dut.WaitForConvReady(10000) == -3) {
            pc.printf("TIMEOUT");
            return;
        }

        dut.ReadData(&data);

        disable_channel(i);


        pc.printf("Channel: %d\r\n", data & 0xff);
        pc.printf("Data reg: %x \r\n", data);
        volt = data_to_voltage(data >> 8);
        pc.printf("Voltage = %f\r\n", volt);
        conversion_results[i] = conversion_fkt[i](volt, cal_current);
        pc.printf("\r\n");

    }
}

void CN0398_Diag::read_volt()
{
    uint8_t regVal = strtol(cmdbuffer[1].c_str(), NULL, 16);
    pc.printf("Data reg: %x ",
              dut.ReadDeviceRegister(
                  static_cast<AD7124::ad7124_registers>(regVal)));
}
*/
