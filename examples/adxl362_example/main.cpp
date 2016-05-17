/**
 *   @file     main.cpp
 *   @brief    Main file for the ADXL362-example project
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
#include "mbed.h"
#include "ADXL362.h"
#include "lcd.h"

Serial pc(USBTX, USBRX); ///< Serial interface to the pc
Lcd lcd;
ADXL362 adxl362(D9);

static const uint16_t INACT_VAL = 50;
static const uint16_t INACT_TIMER = 10 * 100;
static const uint16_t ACT_VAL = 50;
static const uint8_t ACT_TIMER = 100;
static const uint16_t SCAN_SENSOR_TIME = 500;

void flush_serial_buffer(void)
{
    while (pc.readable())
        pc.getc();
    return;
}

int main()
{
    uint8_t ui8s[22];
    uint8_t ui8xu = 0;
    uint8_t ui8xd = 0;
    uint8_t ui8yu = 0;
    uint8_t ui8yd = 0;
    uint8_t ui8all = 0;

    lcd.init();
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
    // interrupt ?

    adxl362.set_mode(ADXL362::MEASUREMENT);

    uint8_t ui8awake = 0;

    /* Infinite loop */
    while (1) {
#if(ADXL_INT_SEL == INTACC_PIN_1)
        if (adxl362.get_int1()) {
#elif(ADXL_INT_SEL == INTACC_PIN_2)
            if(adxl362.get_int2()) {
#endif
                if (ui8awake == 0) {
                    ui8awake = 1;

                    /* Set BLLCD pin - turn on LCD backlight */
                    lcd.bl_enable();

                    lcd.display_string(0, 60, (int8_t *) "[mG]");
                    lcd.display_string(1, 60, (int8_t *) "[mG]");
                    lcd.display_string(2, 60, (int8_t *) "[mG]");

#if TEMP_ADC == 1
                    lcd.display_string(3, 60, (int8_t *)"[ADC]");
#else
                    lcd.display_string(3, 60, (int8_t *) "[C]");
#endif
                    ui8xu = 0;
                    ui8xd = 0;
                    ui8yu = 0;
                    ui8yd = 0;
                    ui8all = 0;

                    lcd.display_symbol(0, Lcd::UP_X, 8, Lcd::pui8RecInv8x8);
                    lcd.display_symbol(1, Lcd::LEFT_X, 8, Lcd::pui8RecInv8x8);
                    lcd.display_symbol(1, Lcd::RIGHT_X, 8, Lcd::pui8RecInv8x8);
                    lcd.display_symbol(2, Lcd::DOWN_X, 8, Lcd::pui8RecInv8x8);
                    lcd.display_symbol(1, Lcd::CENTER_X, 8, Lcd::pui8RecInv8x8);
                }

            } else {
                if (ui8awake == 1) {
                    ui8awake = 0;

                    /* Clear BLLCD pin - turn off LCD backlight */
                    lcd.bl_disable();

                    /* Clear screen */
                    lcd.fill_pages(0, 4, 0x00);
                }
            }

            if (ui8awake == 1) {
                wait_ms(100);
                uint64_t acc = adxl362.scan();
                int16_t i16SensorX = static_cast<uint16_t>((acc & 0xffff000000000000) >> 48);
                int16_t i16SensorY = static_cast<uint16_t>((acc & 0x0000ffff00000000) >> 32);
                int16_t i16SensorZ = static_cast<uint16_t>((acc & 0x00000000ffff0000) >> 16);
                int16_t i16SensorT = static_cast<uint16_t> (acc & 0x000000000000ffff);

                sprintf((char *) ui8s, "x = % 5d", i16SensorX);
                lcd.display_string(0, 0, (int8_t *) ui8s);

                sprintf((char *) ui8s, "y = % 5d", i16SensorY);
                lcd.display_string(1, 0, (int8_t *) ui8s);

                sprintf((char *) ui8s, "z = % 5d", i16SensorZ);
                lcd.display_string(2, 0, (int8_t *) ui8s);

#if TEMP_ADC == 1
                sprintf((char *)ui8s, "t = % 5d", i16SensorT);
                lcd.display_string(3, 0, (int8_t *)ui8s);
#else
                float f32temp = ((float) i16SensorT + Lcd::ACC_TEMP_BIAS)
                                / (1 / Lcd::ACC_TEMP_SENSITIVITY);   // -34.625
                sprintf((char *) ui8s, "t = % 4.1f", f32temp);
                lcd.display_string(3, 0, (int8_t *) ui8s);
#endif

                if (i16SensorY > Lcd::ACC_LIMIT) {
                    if (ui8xu == 0) {
                        ui8xu = 1;
                        lcd.display_symbol(0, Lcd::UP_X, 8, Lcd::pui8Rec8x8);
                    }

                } else {
                    if (ui8xu == 1) {
                        ui8xu = 0;
                        lcd.display_symbol(0, Lcd::UP_X, 8, Lcd::pui8RecInv8x8);
                    }
                }

                if (i16SensorY < -Lcd::ACC_LIMIT) {
                    if (ui8xd == 0) {
                        ui8xd = 1;
                        lcd.display_symbol(2, Lcd::DOWN_X, 8, Lcd::pui8Rec8x8);
                    }

                } else {
                    if (ui8xd == 1) {
                        ui8xd = 0;
                        lcd.display_symbol(2, Lcd::DOWN_X, 8, Lcd::pui8RecInv8x8);
                    }
                }

                if (i16SensorX > Lcd::ACC_LIMIT) {
                    if (ui8yu == 0) {
                        ui8yu = 1;
                        lcd.display_symbol(1, Lcd::RIGHT_X, 8, Lcd::pui8Rec8x8);
                    }

                } else {
                    if (ui8yu == 1) {
                        ui8yu = 0;
                        lcd.display_symbol(1, Lcd::RIGHT_X, 8, Lcd::pui8RecInv8x8);
                    }
                }

                if (i16SensorX < -Lcd::ACC_LIMIT) {
                    if (ui8yd == 0) {
                        ui8yd = 1;
                        lcd.display_symbol(1, Lcd::LEFT_X, 8, Lcd::pui8Rec8x8);
                    }

                } else {
                    if (ui8yd == 1) {
                        ui8yd = 0;
                        lcd.display_symbol(1, Lcd::LEFT_X, 8, Lcd::pui8RecInv8x8);
                    }
                }

                if ((ui8xu == 0) && (ui8xd == 0) && (ui8yu == 0) && (ui8yd == 0)) {
                    if (ui8all == 0) {
                        ui8all = 1;
                        lcd.display_symbol(1, Lcd::CENTER_X, 8, Lcd::pui8Rec8x8);
                    }

                } else {
                    if (ui8all == 1) {
                        ui8all = 0;
                        lcd.display_symbol(1, Lcd::CENTER_X, 8, Lcd::pui8RecInv8x8);
                    }
                }
            }
        }

    }

