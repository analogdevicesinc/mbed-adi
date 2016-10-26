/**
 *   @file     main.cpp
 *   @brief    Main file for the CN0398-example project
 *   @author   Analog Devices Inc.
 *
 * For support please go to:
 * Github: https://github.com/analogdevicesinc/mbed-adi
 * Support: https://ez.analog.com/community/linux-device-drivers/microcontroller-no-os-drivers
 * Product: www.analog.com/EVAL-CN0398-ARDZ
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
#include "CN0398.h"

Serial pc(SERIAL_TX, SERIAL_RX);

/**
 * @brief: Prints calibration solutions
 */
void print_calibration_solutions()
{
    pc.printf("0. ACETATE\n1. BORATE\n2. CAOH2\n3. CARBONATE\n4. CITRATE\n5. HCL\n6. OXALATE\n7. PHOSPHATE0\n8. PHOSPHATE1\n9. PHOSPHATE2\nA. PHTHALATE\nB. TARTRATE\nC. TRIS\nD. pH4\nE. pH 10");
}

/**
 * @brief: flushes the serial interface.
 */
void flush_serial()
{
    wait_ms(10); // wait for all data to come through
    while (pc.readable())  pc.getc();
}

int main()
{
    pc.baud(115200);
    CN0398 cn0398(D10, D4);
    cn0398.reset();
    cn0398.setup();
    cn0398.init();
    wait_ms(500);
    pc.printf("CN0398 - ph and moisture measurement demo\r\nPress any key to continue...\r\n");
    pc.getc();
    wait_ms(10);
    flush_serial();

    pc.printf("Do you want to perform calibration [y/N] ? ");
    char response = pc.getc();
    flush_serial();
    if(response == 'y' || response == 'Y') {
        pc.printf("Do you want to calibrate offset voltage [y/N] ? ");
        response = pc.getc();
        flush_serial();
        if(response == 'y' || response == 'Y') {
            pc.printf("Calibration step 0. Short the pH probe and press any key to calibrate.\r\n");
            pc.getc();
            flush_serial();
            cn0398.calibrate_ph_offset();
        }
        print_calibration_solutions();

        bool response_ok = false;
        while(response_ok == false) {
            pc.printf("\nInput calibration solution used for first step [1-9][a-e] \n");
            char response = pc.getc();
            flush_serial();
            if(response >= '0' && response <= '9')	{
                response_ok = true;
                cn0398.solution0 = response - '0';
            } else if(response >= 'A' && response <= 'E') {
                response_ok = true;
                cn0398.solution0 = response - 'A' + 10;
            } else if(response >= 'a' && response <= 'e') {
                response_ok = true;
                cn0398.solution0 = response - 'a' + 10;
            } else {
                response_ok = false;
            }
        }
        pc.printf("%s solution selected. pH of the solution @ 25 degrees = %f \n", solutions[cn0398.solution0], ph_temp_lut[cn0398.solution0][11]);
        float temperature = cn0398.read_rtd();
        pc.printf("Calibration step 1. Place pH probe in first calibration solution and press any key to start calibration.\r\n");
        pc.getc();
        flush_serial();
        cn0398.calibrate_ph_pt0(temperature);

        response_ok = false;
        while(response_ok == false) {
            pc.printf("Input calibration solution used for second step [1-9][a-e] \n");
            char response = pc.getc();
            flush_serial();
            if(response >= '0' && response <= '9')	{
                response_ok = true;
                cn0398.solution1 = response - '0';
            } else if(response >= 'A' && response <= 'E') {
                response_ok = true;
                cn0398.solution1 = response - 'A' + 10;
            } else if(response >= 'a' && response <= 'e') {
                response_ok = true;
                cn0398.solution1 = response - 'a' + 10;
            } else {
                response_ok = false;
            }
        }
        pc.printf("%s solution selected. pH of the solution @ 25 degrees = %f \n", solutions[cn0398.solution1], ph_temp_lut[cn0398.solution1][11]);
        pc.printf("Calibration step 2. Place pH probe in second calibration solution and press any key to start calibration.\r\n");
        pc.getc();
        flush_serial();
        cn0398.calibrate_ph_pt1(temperature);

    } else {
        cn0398.use_nernst = true;
        pc.printf("Do you want to load default calibration. If not, the Nernst equation will be used [y/N] ? ");
        char response = pc.getc();
        flush_serial();
        if(response == 'y' || response == 'Y') {
            cn0398.use_nernst = false;
        }
    }


    while(1) {
        float temperature = cn0398.read_rtd();
        pc.printf("Temperature: %f\r\n", temperature);
        pc.printf("pH: %f\r\n", cn0398.read_ph(temperature));
#ifdef MOISTURE_SENSOR_PRESENT
        pc.printf("Moisture: %f\r\n", cn0398.read_moist());
#endif
        pc.printf("\r\n");
        wait_ms(1000);
    }
    return 0;
}
