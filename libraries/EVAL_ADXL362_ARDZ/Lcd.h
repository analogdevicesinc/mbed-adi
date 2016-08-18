/**
 ******************************************************************************
 *   @file     Lcd.c
 *   @brief    Header file for ST7565R LCD control.
 *   @author   ADI
 *   @date     March 2016
 *
 *******************************************************************************
 * Copyright 2015(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
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
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 **/
#ifndef LCD_H_
#define LCD_H_


/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdio.h>
#include "mbed.h"


/**
 * @brief Lcd class
 */
class Lcd
{
public:

    const static uint8_t pui8Rec8x8[8];
    const static uint8_t pui8RecInv8x8[8];
    const static uint8_t pui8font5x7[96][5]; ///< Symbol matrix structure: Font (8x14)

    Lcd(PinName rst = D3, PinName a0 = D5, PinName bl = D8, PinName cs = D6,
        PinName MOSI = SPI_MOSI, PinName MISO = SPI_MISO, PinName SCK =
            SPI_SCK);
    void init(void);
    void display_string(uint8_t ui8row, uint8_t ui8col, int8_t *pi8str);
    void display_symbol(uint8_t ui8row, uint8_t ui8col, uint8_t ui8width,
                        const uint8_t *pui8symbol);
    void fill_pages(uint8_t ui8start, uint8_t ui8num, uint8_t ui8Data);
    void set_line(uint8_t ui8line);
    void set_cursor(uint8_t ui8PA, uint8_t ui8CA);
    void bl_enable();
    void bl_disable();

    void write_cmd(uint8_t cmd);
    void write_data(uint8_t data);


    typedef enum {
        CMD_DISPLAY_OFF = 0xAE,
        CMD_DISPLAY_ON = 0xAF,
        CMD_SET_DISP_START_LINE = 0x40,
        CMD_SET_PAGE = 0xB0,
        CMD_SET_COLUMN_UPPER = 0x10,
        CMD_SET_COLUMN_LOWER = 0x00,
        CMD_SET_ADC_NORMAL = 0xA0,
        CMD_SET_ADC_REVERSE = 0xA1,
        CMD_SET_DISP_NORMAL = 0xA6,
        CMD_SET_DISP_REVERSE = 0xA7,
        CMD_SET_ALLPTS_NORMAL = 0xA4,
        CMD_SET_ALLPTS_ON = 0xA5,
        CMD_SET_BIAS_9 = 0xA2,
        CMD_SET_BIAS_7 = 0xA3,
        CMD_RMW = 0xE0,
        CMD_RMW_CLEAR = 0xEE,
        CMD_INTERNAL_RESET = 0xE2,
        CMD_SET_COM_NORMAL = 0xC0,
        CMD_SET_COM_REVERSE = 0xC8,
        CMD_SET_POWER_CONTROL = 0x28,
        CMD_SET_RESISTOR_RATIO = 0x20,
        CMD_SET_VOLUME_FIRST = 0x81,
        CMD_SET_VOLUME_SECOND = 0,
        CMD_SET_STATIC_OFF = 0xAC,
        CMD_SET_STATIC_ON = 0xAD,
        CMD_SET_STATIC_REG = 0x0,
        CMD_SET_BOOSTER_FIRST = 0xF8,
        CMD_SET_BOOSTER_234 = 0,
        CMD_SET_BOOSTER_5 = 1,
        CMD_SET_BOOSTER_6 = 3,
        CMD_NOP = 0xE3,
        CMD_TEST = 0xF0,
    } lcd_commands_t;

    static const uint8_t LCD_COLUMNS = 128u;
    static const uint8_t LCD_PAGES = 4u;
    static const uint8_t LCD_LINES = 64u;
    static const uint8_t UP_X = 112;
    static const uint8_t LEFT_X = 104;
    static const uint8_t RIGHT_X = 120;
    static const uint8_t DOWN_X = 112;
    static const uint8_t CENTER_X = 112;
    static const uint8_t ACC_LIMIT = 80;
    static const uint8_t FONT_Y_SIZE = 8; ///< Font size for Y
    static const uint8_t OFFS_ASCII = 32; ///< ASCII offset

    static const float ACC_TEMP_BIAS          =   350  ;       ///< Accelerometer temperature bias(in ADC codes) at 25 Deg C
    static const float ACC_TEMP_SENSITIVITY   =   0.065;       ///< Accelerometer temperature sensitivity  from datasheet (DegC per Code)

private:

    DigitalOut rst, a0, bl, cs;
    SPI lcd_spi;



};

#endif /* LCD_H_ */
