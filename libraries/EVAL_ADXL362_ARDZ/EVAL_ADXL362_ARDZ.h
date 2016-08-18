/**
 *   @file     EVAL_ADXL362_ARDZ.cpp
 *   @brief    Header file for the EVAL-ADXL362-ARDZ board
 *   @author   Analog Devices Inc.
 *
 * For support please go to:
 * Github: https://github.com/analogdevicesinc/mbed-adi
 * Support: https://ez.analog.com/community/linux-device-drivers/microcontroller-no-os-drivers
 * Product: www.analog.com/EVAL-ADXL362-ARDZ
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

#ifndef EVAL_ADXL362_ARDZ_H
#define EVAL_ADXL362_ARDZ_H

#include "Lcd.h"
#include "ADXL362.h"

/**
 * @brief EVAL-ADXL362_ARDZ accelerometer shield
 */
class EVAL_ADXL362_ARDZ
{
public:
    EVAL_ADXL362_ARDZ(Lcd& _lcd, ADXL362& _adxl362) ;
    void ADXL_setup();
    void ADXL_scan_xyzt();
    bool ADXL_get_int();

    void LCD_setup();
    void LCD_init_display();
    void LCD_deinit_display();
    void LCD_display_level();
    void LCD_display_values();

    Lcd lcd;
    ADXL362 adxl362;

private:

    uint8_t _lcd_on;
    int16_t _x_axis_data;
    int16_t _y_axis_data;
    int16_t _z_axis_data;
    int16_t _t_data;

    static const uint16_t INACT_VAL = 50;
    static const uint16_t INACT_TIMER = 100 * 10;
    static const uint16_t ACT_VAL = 50;
    static const uint8_t ACT_TIMER = 100;
    static const uint16_t SCAN_SENSOR_TIME = 500;
};

#endif
