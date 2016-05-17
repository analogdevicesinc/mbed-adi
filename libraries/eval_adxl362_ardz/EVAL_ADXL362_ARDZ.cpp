/**
 *   @file     EVAL_ADXL362_ARDZ.cpp
 *   @brief    Source file for the EVAL-ADXL362-ARDZ board
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

#include "EVAL_ADXL362_ARDZ.h"

/**
 * Constructor for the  EVAL_ADXL362_ARDZ
 * @param _lcd reference to an LCD object
 * @param _adxl362 reference to an ADXL362 object
 */
EVAL_ADXL362_ARDZ::EVAL_ADXL362_ARDZ(Lcd& _lcd, ADXL362& _adxl362) : lcd(_lcd) , adxl362(_adxl362)
{
    _x_axis_data = 0;
    _y_axis_data = 0;
    _z_axis_data = 0;
    _t_data = 0;
    _lcd_on = 0;
}

/**
 * Initial setup of the LCD
 */
void EVAL_ADXL362_ARDZ::LCD_setup()
{
    lcd.init();
}

/**
 * Initial setup of the ADXL
 */
void EVAL_ADXL362_ARDZ::ADXL_setup()
{
    adxl362.reset();
    wait_us(500);
    adxl362.set_activity_threshold(ACT_VAL);
    adxl362.set_activity_time(ACT_TIMER / 10);

    adxl362.set_inactivity_threshold(INACT_VAL);
    adxl362.set_inactivity_time(INACT_TIMER);
    adxl362.set_act_inact_ctl_reg(0x3f);
#if(ADXL_INT_SEL == INTACC_PIN_1)
    adxl362.set_polling_interrupt1_pin(D2, 0x40);
#elif(ADXL_INT_SEL == INTACC_PIN_2)
    adxl362.set_polling_interrupt2_pin(D2, 0x40); /* Map the awake status to INT2 pin */
#endif
    adxl362.set_mode(ADXL362::MEASUREMENT);
    _lcd_on = 0;

}

/**
 * Method used to scan the ADXL axis data to the internal members
 */
void EVAL_ADXL362_ARDZ::ADXL_scan_xyzt()
{
    uint64_t acc = adxl362.scan();
    _x_axis_data = static_cast<uint16_t>((acc & 0xffff000000000000) >> 48);
    _y_axis_data = static_cast<uint16_t>((acc & 0x0000ffff00000000) >> 32);
    _z_axis_data = static_cast<uint16_t>((acc & 0x00000000ffff0000) >> 16);
    _t_data = static_cast<uint16_t> (acc & 0x000000000000ffff);
}

/**
 * Gets the status of the interrupt
 * @return true if interrupt is active
 */
bool EVAL_ADXL362_ARDZ::ADXL_get_int()
{
#if(ADXL_INT_SEL == INTACC_PIN_1)
    return adxl362.get_int1();
#elif(ADXL_INT_SEL == INTACC_PIN_2)
    return adxl362.get_int2();
#endif
    return false;
}

/**
 * Displays numeric values on the LCD
 */
void EVAL_ADXL362_ARDZ::LCD_display_values()
{
    uint8_t string[22];
    sprintf((char *) string, "x = % 5d", _x_axis_data);
    lcd.display_string(0, 0, (int8_t *) string);

    sprintf((char *) string, "y = % 5d", _y_axis_data);
    lcd.display_string(1, 0, (int8_t *) string);

    sprintf((char *) string, "z = % 5d", _z_axis_data);
    lcd.display_string(2, 0, (int8_t *) string);

#if TEMP_ADC == 1
    sprintf((char *)string, "t = % 5d", _t_data);
    lcd.display_string(3, 0, (int8_t *)string);
#else
    float f32temp = ((float) _t_data + Lcd::ACC_TEMP_BIAS)
                    / (1 / Lcd::ACC_TEMP_SENSITIVITY);   // -34.625
    sprintf((char *) string, "t = % 4.1f", f32temp);
    lcd.display_string(3, 0, (int8_t *) string);
#endif
}

/**
 * Displays the level meter on the LCD
 */
void EVAL_ADXL362_ARDZ::LCD_display_level()
{
    bool any_direction = false;
    lcd.display_symbol(0, Lcd::UP_X, 8, Lcd::pui8RecInv8x8);
    lcd.display_symbol(2, Lcd::DOWN_X, 8, Lcd::pui8RecInv8x8);
    lcd.display_symbol(1, Lcd::LEFT_X, 8, Lcd::pui8RecInv8x8);
    lcd.display_symbol(1, Lcd::RIGHT_X, 8, Lcd::pui8RecInv8x8);

    if(_x_axis_data >  Lcd::ACC_LIMIT) {
        lcd.display_symbol(1, Lcd::RIGHT_X, 8, Lcd::pui8Rec8x8);
        any_direction = true;
    }
    if(_x_axis_data < -Lcd::ACC_LIMIT) {
        lcd.display_symbol(1, Lcd::LEFT_X, 8, Lcd::pui8Rec8x8);
        any_direction = true;
    }
    if(_y_axis_data >  Lcd::ACC_LIMIT) {
        lcd.display_symbol(0, Lcd::DOWN_X, 8, Lcd::pui8Rec8x8);
        any_direction = true;
    }
    if(_y_axis_data < -Lcd::ACC_LIMIT) {
        lcd.display_symbol(2, Lcd::UP_X, 8, Lcd::pui8Rec8x8);
        any_direction = true;
    }
    if( any_direction ) lcd.display_symbol(1, Lcd::CENTER_X, 8, Lcd::pui8RecInv8x8);
    else lcd.display_symbol(1, Lcd::CENTER_X, 8, Lcd::pui8Rec8x8);

}

/**
 * Turns on the backlight and draws the static data to the LCD
 */
void EVAL_ADXL362_ARDZ::LCD_init_display()
{
    /* Set BLLCD pin - turn on LCD backlight */
    if(_lcd_on == true) return;
    lcd.bl_enable();

    lcd.display_string(0, 60, (int8_t *) "[mG]");
    lcd.display_string(1, 60, (int8_t *) "[mG]");
    lcd.display_string(2, 60, (int8_t *) "[mG]");

#if (TEMP_ADC == 1)
    lcd.display_string(3, 60, (int8_t *)"[ADC]");
#else
    lcd.display_string(3, 60, (int8_t *) "[C]");
#endif

    lcd.display_symbol(0, Lcd::UP_X, 8, Lcd::pui8RecInv8x8);
    lcd.display_symbol(1, Lcd::LEFT_X, 8, Lcd::pui8RecInv8x8);
    lcd.display_symbol(1, Lcd::RIGHT_X, 8, Lcd::pui8RecInv8x8);
    lcd.display_symbol(2, Lcd::DOWN_X, 8, Lcd::pui8RecInv8x8);
    lcd.display_symbol(1, Lcd::CENTER_X, 8, Lcd::pui8RecInv8x8);
    _lcd_on = true;
}

/**
 * Turns off the backlight and clears the LCD
 */
void EVAL_ADXL362_ARDZ::LCD_deinit_display()
{
    if(_lcd_on == false) return;
    /* Clear BLLCD pin - turn off LCD backlight */
    lcd.bl_disable();

    /* Clear screen */
    lcd.fill_pages(0, 4, 0x00);
    _lcd_on = false;
}


