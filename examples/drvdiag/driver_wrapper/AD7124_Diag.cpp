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
#include "AD7124_Diag.h"

#include "../../../libraries/Thermocouple/Thermocouple.h"


extern Serial pc;
extern vector<string> cmdbuffer;

//#define CALIBRATION

AD7124_Diag::AD7124_Diag(AD7124& ad) :
    dut(ad)
{

}

void AD7124_Diag::setup()
{
    dut.frequency(500000);
    dut.Setup();
}

void AD7124_Diag::mvpInit()
{
    uint32_t setValue;
    enum AD7124::ad7124_registers regNr;
    dut.frequency(500000);

    /* Set Config_0 0x19*/
    regNr = AD7124::AD7124_Config_0;               //Select Config_0 register
    setValue = dut.ReadDeviceRegister(regNr);
    setValue |= AD7124_CFG_REG_BIPOLAR;     //Select bipolar operation
    setValue |= AD7124_CFG_REG_BURNOUT(0);  //Burnout current source off
    setValue |= AD7124_CFG_REG_REF_BUFP;
    setValue |= AD7124_CFG_REG_REF_BUFM;
    setValue |= AD7124_CFG_REG_AIN_BUFP;    //Buffer AIN5
    setValue |= AD7124_CFG_REG_AINN_BUFM;   //Buffer AIN4
    setValue |= AD7124_CFG_REG_REF_SEL(2); //Select REFIN1(+)/REFIN1(-)  internal reference
    setValue |= AD7124_CFG_REG_PGA(0);
    setValue &= 0xFFFF;
    dut.WriteDeviceRegister(regNr, setValue);   // Write data to ADC

    /* Set Channel_0 register 0x09*/
    regNr = AD7124::AD7124_Channel_0;
    setValue = dut.ReadDeviceRegister(regNr);
    //setValue |= (uint32_t) AD7124_CH_MAP_REG_CH_ENABLE;  //Enable channel0
    setValue |= AD7124_CH_MAP_REG_SETUP(0);             // Select setup0
    setValue |= AD7124_CH_MAP_REG_AINP(0);         // Set AIN4 as positive input
    setValue |= AD7124_CH_MAP_REG_AINM(15);         // Set AIN5 as negative input
    setValue &= 0xFFFF;
    dut.WriteDeviceRegister(regNr, setValue);   // Write data to ADC

    regNr = AD7124::AD7124_Channel_1;
    setValue = dut.ReadDeviceRegister(regNr);
    //setValue |= (uint32_t) AD7124_CH_MAP_REG_CH_ENABLE;  //Enable channel0
    setValue |= AD7124_CH_MAP_REG_SETUP(0);             // Select setup0
    setValue |= AD7124_CH_MAP_REG_AINP(1);         // Set AIN4 as positive input
    setValue |= AD7124_CH_MAP_REG_AINM(15);         // Set AIN5 as negative input
    setValue &= 0xFFFF;
    dut.WriteDeviceRegister(regNr, setValue);   // Write data to ADC

    regNr = AD7124::AD7124_Channel_2;
    setValue = dut.ReadDeviceRegister(regNr);
    //setValue |= (uint32_t) AD7124_CH_MAP_REG_CH_ENABLE;  //Enable channel0
    setValue |= AD7124_CH_MAP_REG_SETUP(0);             // Select setup0
    setValue |= AD7124_CH_MAP_REG_AINP(2);         // Set AIN4 as positive input
    setValue |= AD7124_CH_MAP_REG_AINM(15);         // Set AIN5 as negative input
    setValue &= 0xFFFF;
    dut.WriteDeviceRegister(regNr, setValue);   // Write data to ADC

    regNr = AD7124::AD7124_Channel_3;
    setValue = dut.ReadDeviceRegister(regNr);
    //setValue |= (uint32_t) AD7124_CH_MAP_REG_CH_ENABLE;  //Enable channel0
    setValue |= AD7124_CH_MAP_REG_SETUP(0);             // Select setup0
    setValue |= AD7124_CH_MAP_REG_AINP(3);         // Set AIN4 as positive input
    setValue |= AD7124_CH_MAP_REG_AINM(15);         // Set AIN5 as negative input
    setValue &= 0xFFFF;
    dut.WriteDeviceRegister(regNr, setValue);   // Write data to ADC

    regNr = AD7124::AD7124_Channel_4;
    setValue = dut.ReadDeviceRegister(regNr);
    //setValue |= (uint32_t) AD7124_CH_MAP_REG_CH_ENABLE;  //Enable channel0
    setValue |= AD7124_CH_MAP_REG_SETUP(0);             // Select setup0
    setValue |= AD7124_CH_MAP_REG_AINP(4);         // Set AIN4 as positive input
    setValue |= AD7124_CH_MAP_REG_AINM(15);         // Set AIN5 as negative input
    setValue &= 0xFFFF;
    dut.WriteDeviceRegister(regNr, setValue);   // Write data to ADC


    regNr = AD7124::AD7124_Channel_5;
    setValue = dut.ReadDeviceRegister(regNr);
    //setValue |= (uint32_t) AD7124_CH_MAP_REG_CH_ENABLE;  //Enable channel0
    setValue |= AD7124_CH_MAP_REG_SETUP(0);             // Select setup0
    setValue |= AD7124_CH_MAP_REG_AINP(5);         // Set AIN4 as positive input
    setValue |= AD7124_CH_MAP_REG_AINM(15);         // Set AIN5 as negative input
    setValue &= 0xFFFF;
    dut.WriteDeviceRegister(regNr, setValue);   // Write data to ADC


    regNr = AD7124::AD7124_Channel_6;
    setValue = dut.ReadDeviceRegister(regNr);
    //setValue |= (uint32_t) AD7124_CH_MAP_REG_CH_ENABLE;  //Enable channel0
    setValue |= AD7124_CH_MAP_REG_SETUP(0);             // Select setup0
    setValue |= AD7124_CH_MAP_REG_AINP(6);         // Set AIN4 as positive input
    setValue |= AD7124_CH_MAP_REG_AINM(15);         // Set AIN5 as negative input
    setValue &= 0xFFFF;
    dut.WriteDeviceRegister(regNr, setValue);   // Write data to ADC

    regNr = AD7124::AD7124_Channel_7;
    setValue = dut.ReadDeviceRegister(regNr);
    //setValue |= (uint32_t) AD7124_CH_MAP_REG_CH_ENABLE;  //Enable channel0
    setValue |= AD7124_CH_MAP_REG_SETUP(0);             // Select setup0
    setValue |= AD7124_CH_MAP_REG_AINP(7);         // Set AIN4 as positive input
    setValue |= AD7124_CH_MAP_REG_AINM(15);         // Set AIN5 as negative input
    setValue &= 0xFFFF;
    dut.WriteDeviceRegister(regNr, setValue);   // Write data to ADC

    regNr = AD7124::AD7124_Channel_8;
    setValue = dut.ReadDeviceRegister(regNr);
    //setValue |= (uint32_t) AD7124_CH_MAP_REG_CH_ENABLE;  //Enable channel0
    setValue |= AD7124_CH_MAP_REG_SETUP(0);             // Select setup0
    setValue |= AD7124_CH_MAP_REG_AINP(14);         // Set AIN4 as positive input
    setValue |= AD7124_CH_MAP_REG_AINM(15);         // Set AIN5 as negative input
    setValue &= 0xFFFF;
    dut.WriteDeviceRegister(regNr, setValue);   // Write data to ADC


    /* Set Config_0 0x19*/
#if 0
    regNr = AD7124::AD7124_Config_1;               //Select Config_0 register
    setValue = dut.ReadDeviceRegister(regNr);
    setValue |= AD7124_CFG_REG_BIPOLAR;     //Select bipolar operation
    setValue |= AD7124_CFG_REG_BURNOUT(0);  //Burnout current source off
    setValue |= AD7124_CFG_REG_REF_BUFP;
    setValue |= AD7124_CFG_REG_REF_BUFM;
    setValue |= AD7124_CFG_REG_AIN_BUFP;    //Buffer AIN5
    setValue |= AD7124_CFG_REG_AINN_BUFM;   //Buffer AIN4
    setValue |= AD7124_CFG_REG_REF_SEL(2); //Select REFIN1(+)/REFIN1(-)  internal reference
    setValue &= 0xFFFF;
    dut.WriteDeviceRegister(regNr, setValue);   // Write data to ADC
#endif

#ifdef  CALIBRATION
    // start calibration
    regNr = AD7124::AD7124_Offset_0;
    setValue = 0x800000;
    dut.WriteDeviceRegister(regNr, setValue);// Write data to ADC

// internal fullscale before zero scale
    pc.printf("\r\n Gain before cali :%x", dut.ReadDeviceRegister(AD7124::AD7124_Gain_0));
    regNr = AD7124::AD7124_ADC_Control;//Select ADC_Control register
    setValue = AD7124_ADC_CTRL_REG_MODE(6);
    setValue |= AD7124_ADC_CTRL_REG_REF_EN;
    setValue &= 0xFFFF;
    dut.WriteDeviceRegister(regNr, setValue);// Write data to ADC
    //dut.WaitForConvReady(10000);
    wait_ms(2000);

    pc.printf("\r\n Gain:%x", dut.ReadDeviceRegister(AD7124::AD7124_Gain_0));

    pc.printf("\r\n Offset before cali:%x", dut.ReadDeviceRegister(AD7124::AD7124_Offset_0));
// internal zeroscale
    regNr = AD7124::AD7124_ADC_Control;//Select ADC_Control register
    setValue = AD7124_ADC_CTRL_REG_MODE(5);
    setValue |= AD7124_ADC_CTRL_REG_REF_EN;
    setValue &= 0xFFFF;
    dut.WriteDeviceRegister(regNr, setValue);// Write data to ADC
    wait_ms(2000);
    pc.printf("\r\n Offset:%x\r\n", dut.ReadDeviceRegister(AD7124::AD7124_Offset_0));

    // end of calibration

#endif

    /* Set IO_Control_1 0x03 */
    regNr = AD7124::AD7124_IOCon1;               //Select IO_Control_1 register
    setValue = dut.ReadDeviceRegister(regNr);
    setValue |= AD7124_IO_CTRL1_REG_IOUT0(0x4);// set IOUT0 current to 500uA
    setValue |= AD7124_IO_CTRL1_REG_IOUT_CH0(0x1);
    setValue &= 0xFFFFFF;
    dut.WriteDeviceRegister(regNr, setValue);// Write data to ADC
#if 0
    setValue = 0;
    regNr = AD7124::AD7124_Channel_1;

    /* Set Channel_1 register 0x0A*/

    setValue = dut.ReadDeviceRegister( regNr );
    setValue |= (uint32_t)AD7124_CH_MAP_REG_CH_ENABLE;//Enable channel1
    setValue |= AD7124_CH_MAP_REG_SETUP(1);// Select setup1
    setValue |= AD7124_CH_MAP_REG_AINP(2);// Set AIN2 as positive input
    setValue |= AD7124_CH_MAP_REG_AINM(1);// Set AIN1 as negative input
    setValue &= 0xFFFF;
    dut.WriteDeviceRegister(regNr, setValue);// Write data to ADC

    /* Set Config_1 0x1A*/
    regNr = AD7124::AD7124_Config_1;               //Select Config_1 register
    setValue = dut.ReadDeviceRegister(regNr);
    setValue |= AD7124_CFG_REG_BIPOLAR;//Select bipolar operation
    setValue |= AD7124_CFG_REG_BURNOUT(0);//Burnout current source off
    setValue |= AD7124_CFG_REG_REF_BUFP;
    setValue |= AD7124_CFG_REG_REF_BUFM;
    setValue |= AD7124_CFG_REG_AIN_BUFP;//Buffer AIN2
    setValue |= AD7124_CFG_REG_AINN_BUFM;//Buffer AIN1
    setValue |= AD7124_CFG_REG_REF_SEL(2);//Select internal reference
    setValue &= 0xFFFF;
    dut.WriteDeviceRegister(regNr, setValue);// Write data to ADC
#endif

    /* Set ADC_Control 0x01 */
    regNr = AD7124::AD7124_ADC_Control;            //Select ADC_Control register
    setValue = dut.ReadDeviceRegister(regNr);
    setValue |= AD7124_ADC_CTRL_REG_DATA_STATUS; // set data status bit in order to check on which channel the conversion is
    setValue |= AD7124_ADC_CTRL_REG_REF_EN;
    setValue &= 0xFFC3;
    setValue |= AD7124_ADC_CTRL_REG_MODE(1);
    setValue &= 0xFFFF;
    dut.WriteDeviceRegister(regNr, setValue);    // Write data to ADC
}

void AD7124_Diag::write_reg()
{
    uint8_t reg = strtol(cmdbuffer[1].c_str(), NULL, 16);
    uint32_t regData = strtol(cmdbuffer[2].c_str(), NULL, 16);
    //dut.write_reg(reg, regData);
    dut.WriteDeviceRegister(static_cast<AD7124::ad7124_registers>(reg),
                            regData);
    pc.printf("Wrote mode");
}
void AD7124_Diag::read_reg()
{
    uint8_t regVal = strtol(cmdbuffer[1].c_str(), NULL, 16);
    pc.printf("Mode reg: %x ",
              dut.ReadDeviceRegister(
                  static_cast<AD7124::ad7124_registers>(regVal)));
}
void AD7124_Diag::reset()
{
    dut.frequency(500000);
    dut.Reset();
    pc.printf("Reseted AD7124");
}

float AD7124_Diag::data_to_voltage(uint32_t data)
{
    data = data & 0xFFFFFF;
    return ((data / static_cast<float>(0xFFFFFF / 2)) - 1) * (2.5 / 1);
}

void AD7124_Diag::enable_channel(int channel)
{
    AD7124::ad7124_registers regNr = static_cast<AD7124::ad7124_registers> (AD7124::AD7124_Channel_0 + channel); //Select ADC_Control register
    uint32_t setValue = dut.ReadDeviceRegister(regNr);
    setValue |= (uint32_t) AD7124_CH_MAP_REG_CH_ENABLE;  //Enable channel0
    setValue &= 0xFFFF;
    dut.WriteDeviceRegister(regNr, setValue);    // Write data to ADC
}

void AD7124_Diag::disable_channel(int channel)
{
    AD7124::ad7124_registers regNr = static_cast<AD7124::ad7124_registers> (AD7124::AD7124_Channel_0 + channel); //Select ADC_Control register
    uint32_t setValue = dut.ReadDeviceRegister(regNr);
    setValue &= (~(uint32_t) AD7124_CH_MAP_REG_CH_ENABLE);  //Enable channel0
    setValue &= 0xFFFF;
    dut.WriteDeviceRegister(regNr, setValue);    // Write data to ADC
}

void AD7124_Diag::enable_current_source(int current_source_channel)
{
    AD7124::ad7124_registers regNr = AD7124::AD7124_IOCon1; //Select ADC_Control register
    uint32_t setValue = dut.ReadDeviceRegister(regNr);
    setValue &= ~(AD7124_IO_CTRL1_REG_IOUT_CH0(0xF));
    setValue |= AD7124_IO_CTRL1_REG_IOUT_CH0(current_source_channel);// set IOUT0 current to 500uA
    setValue &= 0xFFFFFF;

    dut.WriteDeviceRegister(regNr, setValue);    // Write data to ADC
    pc.printf("Enabled 500uA current source on channel %d\r\n", current_source_channel);
}


#define R2 (1600.0)
#define RTD_SLOPE (15/39.0)
#define RTD_CONSTANT (1039.0)
#define RTD_CONVERSION(R1) RTD_SLOPE * (R1-RTD_CONSTANT)
float convert_rtd(float volt, float current)
{
    float R1 = (volt / current) - R2;
    float temp = RTD_CONVERSION(R1);
    pc.printf("Resistance of RTD is: %f\r\n", R1);
    pc.printf("Temperature is: %f\r\n", temp);
//	pc.printf("mV equivalent(poly method) of thermocouple is %f \r\n",Thermocouple_Type_E::convert_inv(temp));
//	pc.printf("mV equivalent(lut method) of thermocouple is %f \r\n",Thermocouple_Type_E::lookup_inv(temp));

    return 0;
}

float cal_current(float volt, float current)
{
    float cal_current = volt / R2;
    pc.printf("Calibrated current = %f mA \r\n", (cal_current * 1000.0));
    return cal_current;
}

float dummy(float volt, float current)
{
    /* Nothing to do */
    return volt;
}

float convert_thermocouple_e(float volt, float current)
{
    float mv = volt * 1000;
    pc.printf("Voltage in mv = %f\r\n", mv);
//	pc.printf("Temperature(lookup) in celsius = %f\r\n", Thermocouple_Type_E::lookup(mv));
//	pc.printf("Temperature(poly) in celsius = %f\r\n\r\n", Thermocouple_Type_E::convert(mv));
    return 0;
}

float convert_thermocouple_k(float volt, float current)
{
    float mv = volt * 1000;
    pc.printf("Voltage in mv = %f\r\n", mv);
//	pc.printf("Temperature(lookup) in celsius = %f\r\n", Thermocouple_Type_K::lookup(mv));
//	pc.printf("Temperature(poly) in celsius = %f\r\n\r\n", Thermocouple_Type_K::convert(mv));
    return 0;
}


float (*conversion_fkt[9])(float volt, float current) = {convert_thermocouple_e, convert_rtd, convert_thermocouple_k, convert_rtd, dummy, convert_rtd, dummy, convert_rtd, cal_current};
float conversion_results[9];

void AD7124_Diag::start_single_conversion()
{
    AD7124::ad7124_registers regNr = AD7124::AD7124_ADC_Control; //Select ADC_Control register
    uint32_t setValue = dut.ReadDeviceRegister(regNr);
    setValue &= 0xFFC3;
    setValue |= 0x04;               //single conversion;
    setValue |= 0x1600;
    setValue &= 0xFFFF;
    dut.WriteDeviceRegister(regNr, setValue);    // Write data to ADC*/
    wait_ms(1);
}

void AD7124_Diag::read_data()
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

void AD7124_Diag::read_volt()
{
    uint8_t regVal = strtol(cmdbuffer[1].c_str(), NULL, 16);
    pc.printf("Data reg: %x ",
              dut.ReadDeviceRegister(
                  static_cast<AD7124::ad7124_registers>(regVal)));
}
