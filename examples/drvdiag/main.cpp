/**
*   @file     main.cpp
*   @brief    Main file of the driver diag tool
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
#include "stdio.h"
#include <iostream>
#include <string>
#include <vector>

#include "config.h"

Serial pc(SERIAL_TX, SERIAL_RX);
vector<string> cmdbuffer;

class commands
{
public:
    commands(string str, int p, void (*h)() ) : cmd_str(str), nr_of_param(p), fktPtr(h)	{	}
    const static int VAR = -1; // variable number of params
    string cmd_str;
    int nr_of_param;
    void (*fktPtr)(void);
};

// *INDENT-OFF*
const vector<commands> cmdlist = {
		{"nop" ,            0, [](){ }},
		{"echo",commands::VAR, [](){ 	for(auto i = begin(cmdbuffer) + 1, e = end(cmdbuffer); i!=e; ++i) 
										printf("%s ", i->c_str()); printf("\r\n");}},

		{"help",           0 , [](){ 	for(auto command : cmdlist) 
										pc.printf("%s ",command.cmd_str.c_str()); }},
		/* #### SPI ####*/
#ifdef SPI_LOW_LEVEL
		{"csa",             0, [](){ 	spibus.format(8, 3);CSA_pin = !CSA_pin;  wait_us(2); 
									 	pc.printf("CS ADC pin set %s", ((CSA_pin.read()) ? "high" : "low")  ); }},
		{"csr",             0, [](){ 	spibus.format(8, 1);CSR_pin = !CSR_pin;  wait_us(2); 
										pc.printf("CS RDAC pin set %s", ((CSR_pin.read()) ? "high" : "low")  );} },
		{"spi",             1, [](){ 	uint8_t spibyte =  strtol(cmdbuffer[1].c_str(), NULL, 16);
									    pc.printf("writing 0x%x to SPI", spibyte);
									    pc.printf("\r\nreturned: 0x%x ", spibus.write(spibyte)); }},
#endif

#ifdef AD7791_PRESENT
		{"adrst",        0,   [](){ad7791diag.reset();}},
		{"adwrm",        1,   [](){ad7791diag.write_mode();}},
		{"adrdm",        0,   [](){ad7791diag.read_mode();}},
		{"adwrf",        1,   [](){ad7791diag.write_filter();}},
		{"adrdf",        0,   [](){ad7791diag.read_filter();}},
		{"adrdd",        0,   [](){ad7791diag.read_data();}},
		{"adrds",        0,   [](){ad7791diag.read_status();}},
		{"adread32",     0,   [](){ad7791diag.read();}},
		{"adread",       0,   [](){ad7791diag.read_u16();}},
		{"adreadv",      0,   [](){ad7791diag.read_voltage();}},
		{"adsetc",       1,   [](){ad7791diag.set_continous_mode();}},
		{"adsetref",     1,   [](){ad7791diag.set_reference_voltage();}},
		{"adsetch",      1,   [](){ad7791diag.set_channel();}},
#endif

#ifdef CN0216_PRESENT
        {"cninit" ,      1,   [](){cn0216diag.init();}},
        {"cncal"  ,      1,   [](){cn0216diag.calibrate();}},
        {"cnrdw"  ,      0,   [](){cn0216diag.read_weight();}},
#endif

#ifdef AD7790_PRESENT
		{"adrst",        0,   [](){ad7790diag.reset();}},
		{"adwrm",        1,   [](){ad7790diag.write_mode();}},
		{"adrdm",        0,   [](){ad7790diag.read_mode();}},
		{"adwrf",        1,   [](){ad7790diag.write_filter();}},
		{"adrdf",        0,   [](){ad7790diag.read_filter();}},
		{"adrdd",        0,   [](){ad7790diag.read_data();}},
		{"adrds",        0,   [](){ad7790diag.read_status();}},
		{"adread",       0,   [](){ad7790diag.read_u16();}},
		{"adreadv",      0,   [](){ad7790diag.read_voltage();}},
		{"adsetc",       1,   [](){ad7790diag.set_continous_mode();}},
		{"adsetref",     1,   [](){ad7790diag.set_reference_voltage();}},
		{"adsetch",      1,   [](){ad7790diag.set_channel();}},
#endif

#ifdef AD5270_PRESENT
		{"rdwrr" ,       1,   [](){ad5270diag.write_RDAC();}},
		{"rdrdr" ,       0,   [](){ad5270diag.read_RDAC();}},
		{"rdwrcmd" ,     2,   [](){ad5270diag.write_cmd();}},
		{"rdsetz" ,      0,   [](){ad5270diag.set_HiZ();}},
		{"rd50en",       0,   [](){ad5270diag.enable_50TP_programming();}},
		{"rd50ds",       0,   [](){ad5270diag.disable_50TP_programming();}},
		{"rd50st",       0,   [](){ad5270diag.store_50TP();}},
		{"rd50a" ,       0,   [](){ad5270diag.read_50TP_last_address();}},
		{"rd50m" ,       1,   [](){ad5270diag.read_50TP_memory();}},
		{"rdwrc" ,       1,   [](){ad5270diag.write_ctrl_reg();}},
		{"rdrdc" ,       0,   [](){ad5270diag.read_ctrl_reg();}},
		{"rdrst" ,       0,   [](){ad5270diag.reset_RDAC();}},
		{"rdchm" ,       1,   [](){ad5270diag.change_mode();}},
		{"rdwrw" ,       1,   [](){ad5270diag.write_wiper_reg();}},
		{"rdrdw" ,       0,   [](){ad5270diag.read_wiper_reg();}},
#endif

#ifdef CN0357_PRESENT
		{"cnwrdac" ,      1,   [](){cn0357diag.set_RDAC();}},
		{"cnrppm"  ,      0,   [](){cn0357diag.read_ppm();}},
		{"cnparam"  ,      2,   [](){cn0357diag.set_sensor_param();}}
#endif

};
// *INDENT-ON*



void read_from_console()
{
    char buffer[100] = {0};
    size_t readPosition = 0;

    // read from console until newline
    while(1) {
        buffer[readPosition] = pc.getc();
        if(buffer[readPosition] == '\n' || buffer[readPosition] == '\r') {
            buffer[readPosition] = ' ';
            break;
        }
        readPosition++;
    }
    readPosition++;
    buffer[readPosition] = '\0';
    //create std::string from char buffer
    string s(buffer);

    // create std::vector of std:string, each string contains parameter
    size_t pos = 0;
    string delimiter = " ";
    string token;
    while ((pos = s.find(delimiter)) != std::string::npos) {
        token = s.substr(0, pos);
        cmdbuffer.push_back(token);
        s.erase(0, pos + delimiter.length());
    }

}

void run_command()
{
    for(auto i : cmdlist) {
        if(i.cmd_str == cmdbuffer[0]) {
            if(static_cast<int>(cmdbuffer.size()) - 1 == i.nr_of_param || i.nr_of_param == commands::VAR) {
                pc.printf("RX> ");
                i.fktPtr();
            } else {
                pc.printf("RX> ");
                pc.printf("Command %s expects %d parameters, found %d", i.cmd_str.c_str(), i.nr_of_param, cmdbuffer.size() - 1);
            }
            return;
        }
    }
    pc.printf("RX> ");
    pc.printf("Command %s not found", cmdbuffer[0].c_str());

}

int main()

{
    ad7791.frequency(100000);
    pc.printf("\r\n#### DrvDiag ####\r\n");

    while(1) {
        pc.printf("\r\nTX> ");
        read_from_console();
        run_command();
        cmdbuffer.clear();
    }

}

