/**
 ******************************************************************************
 *   @file     Lcd.c
 *   @brief    Source file for ST7565R LCD control.
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

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "mbed.h"
#include "lcd.h"



Lcd::Lcd(PinName rst_pin, PinName a0_pin, PinName bl_pin,
         PinName cs_pin, PinName MOSI,
         PinName MISO, PinName SCK) :

    rst(rst_pin), a0(a0_pin), bl(bl_pin), cs(cs_pin), lcd_spi(MOSI, MISO, SCK)
{
    rst = 1;
}

/**
 @brief Initialization of LCD screen
 @return none
 **/

void Lcd::write_cmd(uint8_t cmd)
{
    cs = 0;
    a0 = 0;
    lcd_spi.write(cmd);
    cs = 1;
}

void Lcd::write_data(uint8_t data)
{
    cs = 0;
    a0 = 1;
    lcd_spi.write(data);
    cs = 1;
}

void Lcd::bl_enable()
{
    bl = 1;
}

void Lcd::bl_disable()
{
    bl = 0;
}

void Lcd::init(void)
{

    write_cmd(CMD_DISPLAY_OFF);
    write_cmd(CMD_SET_BIAS_7);
    write_cmd(CMD_SET_ADC_NORMAL);
    write_cmd(CMD_SET_COM_REVERSE);
    write_cmd((CMD_SET_RESISTOR_RATIO | 0x02));
    write_cmd(CMD_SET_VOLUME_FIRST);
    write_cmd((CMD_SET_VOLUME_SECOND | 0x04));
    write_cmd((CMD_SET_POWER_CONTROL | 0x07));
    fill_pages(0, 8, 0x00);

}

/**
 @brief Displays a string at the specified position for with 5x7 font size.
 @return none
 **/
void Lcd::display_string(uint8_t ui8row, uint8_t ui8col, int8_t *pi8str)
{
    uint8_t ui8x;
    uint8_t ui8i;
    uint8_t ui8ch;
    uint8_t ui8data;

    ui8ch = 0;
    ui8x = ui8col;

    while ((pi8str[ui8ch] != 0) && (ui8col < LCD_COLUMNS)) {
        set_cursor(ui8row, ui8x); /* Set cursor position */

        for (ui8i = 0; ui8i < 5; ui8i++) { /* Symbol matrix column loop */
            ui8data = pui8font5x7[pi8str[ui8ch] - OFFS_ASCII][ui8i];

            write_data(ui8data);
        }

        ui8x += 6; /* Increase column counter with 6 pixels */
        ui8ch++; /* Increment counter */
    }
    write_cmd(CMD_DISPLAY_ON);

}

/**
 @brief Displays a symbol (8 x width) at the specified position on the LCD.
 @param ui8row - row number
 @param ui8col - column number
 @param ui8width - symbol width
 @param pui8symbol - symbol to display
 @return none
 **/

void Lcd::display_symbol(uint8_t ui8row, uint8_t ui8col, uint8_t ui8width,
                         const uint8_t *pui8symbol)
{
    uint8_t ui8i;
    uint8_t ui8data;

    set_cursor(ui8row, ui8col); /* Set cursor position */
    for (ui8i = 0; ui8i < ui8width; ui8i++) { /* Symbol matrix column loop */
        ui8data = pui8symbol[ui8i];
        write_data(ui8data);
    }
    write_cmd(CMD_DISPLAY_ON);
}

/**
 @brief Fills the selected LCD pages with the data specified.
 @param ui8start - start element
 @param ui8num - elements numbers to fill
 @param ui8Data - data to fill
 @return none
 **/

void Lcd::fill_pages(uint8_t ui8start, uint8_t ui8num, uint8_t ui8Data)
{

    uint8_t ui8p;
    uint8_t ui8c;

    for (ui8p = ui8start; ui8p < (ui8start + ui8num); ui8p++) {
        set_cursor(ui8p, 0);

        for (ui8c = 0; ui8c < LCD_COLUMNS; ui8c++) {
            write_data(ui8Data);
        }
    }

    write_cmd(CMD_DISPLAY_ON);
}

/**
 @brief Sets the start line of the LCD.
 @param ui8line - line to start with
 @return none
 **/

void Lcd::set_line(uint8_t ui8line)
{

    uint8_t ui8Cmd;
    ui8Cmd = CMD_SET_DISP_START_LINE | (ui8line & 0x3F); /* Set start line  */
    write_cmd(ui8Cmd);

}


/**
 @brief Sets the cursor position at which data will be written.
 @param ui8PA - page number
 @param ui8CA - column number
 @return none
 **/

void Lcd::set_cursor(uint8_t ui8PA, uint8_t ui8CA)
{
    uint8_t ui8Cmd;

    ui8Cmd = 0xB0 | (ui8PA & 0x0F); /* Set page address */
    write_cmd(ui8Cmd);

    ui8Cmd = ui8CA & 0x0F; /* Set column address LSB CA[3:0] */
    write_cmd(ui8Cmd);

    ui8Cmd = 0x10 | (ui8CA >> 4); /* Set column address MSB CA[7:4] */
    write_cmd(ui8Cmd);
}


const uint8_t Lcd::pui8Rec8x8[8] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};
const uint8_t Lcd::pui8RecInv8x8[8] = {
    0xFF, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0xFF
};

const uint8_t Lcd::pui8font5x7[96][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00},  /*     32  */
    {0x00, 0x00, 0x4F, 0x00, 0x00},  /*  !  33  */
    {0x00, 0x07, 0x00, 0x07, 0x00},  /*  "  34  */
    {0x14, 0x7F, 0x14, 0x7F, 0x14},  /*  #  35  */
    {0x24, 0x2A, 0x7F, 0x2A, 0x12},  /*  $  36  */
    {0x23, 0x13, 0x08, 0x64, 0x62},  /*  %  37  */
    {0x36, 0x49, 0x55, 0x22, 0x50},  /*  &  38  */
    {0x00, 0x05, 0x03, 0x00, 0x00},  /*  '  39  */
    {0x00, 0x1C, 0x22, 0x41, 0x00},  /*  (  40  */
    {0x00, 0x41, 0x22, 0x1C, 0x00},  /*  )  41  */
    {0x14, 0x08, 0x3E, 0x08, 0x14},  /*  *  42  */
    {0x08, 0x08, 0x3E, 0x08, 0x08},  /*  +  43  */
    {0x00, 0x50, 0x30, 0x00, 0x00},  /*  ,  44  */
    {0x08, 0x08, 0x08, 0x08, 0x08},  /*  -  45  */
    {0x00, 0x60, 0x60, 0x00, 0x00},  /*  .  46  */
    {0x20, 0x10, 0x08, 0x04, 0x02},  /*  /  47  */
    {0x3E, 0x51, 0x49, 0x45, 0x3E},  /*  0  48  */
    {0x00, 0x42, 0x7F, 0x40, 0x00},  /*  1  49  */
    {0x42, 0x61, 0x51, 0x49, 0x46},  /*  2  50  */
    {0x21, 0x41, 0x45, 0x4B, 0x31},  /*  3  51  */
    {0x18, 0x14, 0x12, 0x7F, 0x10},  /*  4  52  */
    {0x27, 0x45, 0x45, 0x45, 0x39},  /*  5  53  */
    {0x3C, 0x4A, 0x49, 0x49, 0x30},  /*  6  54  */
    {0x01, 0x71, 0x09, 0x05, 0x03},  /*  7  55  */
    {0x36, 0x49, 0x49, 0x49, 0x36},  /*  8  56  */
    {0x06, 0x49, 0x49, 0x29, 0x1E},  /*  9  57  */
    {0x36, 0x36, 0x00, 0x00, 0x00},  /*  :  58  */
    {0x56, 0x36, 0x00, 0x00, 0x00},  /*  ;  59  */
    {0x08, 0x14, 0x22, 0x41, 0x00},  /*  <  60  */
    {0x14, 0x14, 0x14, 0x14, 0x14},  /*  =  61  */
    {0x00, 0x41, 0x22, 0x14, 0x08},  /*  >  62  */
    {0x02, 0x01, 0x51, 0x09, 0x06},  /*  ?  63  */
    {0x30, 0x49, 0x79, 0x41, 0x3E},  /*  @  64  */
    {0x7E, 0x11, 0x11, 0x11, 0x7E},  /*  A  65  */
    {0x7F, 0x49, 0x49, 0x49, 0x36},  /*  B  66  */
    {0x3E, 0x41, 0x41, 0x41, 0x22},  /*  C  67  */
    {0x7F, 0x41, 0x41, 0x22, 0x1C},  /*  D  68  */
    {0x7F, 0x49, 0x49, 0x49, 0x41},  /*  E  69  */
    {0x7F, 0x09, 0x09, 0x09, 0x01},  /*  F  70  */
    {0x3E, 0x41, 0x49, 0x49, 0x7A},  /*  G  71  */
    {0x7F, 0x08, 0x08, 0x08, 0x7F},  /*  H  72  */
    {0x00, 0x41, 0x7F, 0x41, 0x00},  /*  I  73  */
    {0x20, 0x40, 0x41, 0x3F, 0x01},  /*  J  74  */
    {0x7F, 0x08, 0x14, 0x22, 0x41},  /*  K  75  */
    {0x7F, 0x40, 0x40, 0x40, 0x40},  /*  L  76  */
    {0x7F, 0x02, 0x0C, 0x02, 0x7F},  /*  M  77  */
    {0x7F, 0x04, 0x08, 0x10, 0x7F},  /*  N  78  */
    {0x3E, 0x41, 0x41, 0x41, 0x3E},  /*  O  79  */
    {0x7F, 0x09, 0x09, 0x09, 0x06},  /*  P  80  */
    {0x3E, 0x41, 0x51, 0x21, 0x5E},  /*  Q  81  */
    {0x7F, 0x09, 0x19, 0x29, 0x46},  /*  R  82  */
    {0x46, 0x49, 0x49, 0x49, 0x31},  /*  S  83  */
    {0x01, 0x01, 0x7F, 0x01, 0x01},  /*  T  84  */
    {0x3F, 0x40, 0x40, 0x40, 0x3F},  /*  U  85  */
    {0x1F, 0x20, 0x40, 0x20, 0x1F},  /*  V  86  */
    {0x3F, 0x40, 0x30, 0x40, 0x3F},  /*  W  87  */
    {0x63, 0x14, 0x08, 0x14, 0x63},  /*  X  88  */
    {0x07, 0x08, 0x70, 0x08, 0x07},  /*  Y  89  */
    {0x61, 0x51, 0x49, 0x45, 0x43},  /*  Z  90  */
    {0x00, 0x7F, 0x41, 0x41, 0x00},  /*  [  91  */
    {0x02, 0x04, 0x08, 0x10, 0x20},  /*  \  92  */
    {0x00, 0x41, 0x41, 0x7F, 0x00},  /*  ]  93  */
    {0x04, 0x02, 0x01, 0x02, 0x04},  /*  ^  94  */
    {0x40, 0x40, 0x40, 0x40, 0x40},  /*  _  95  */
    {0x00, 0x01, 0x02, 0x04, 0x00},  /*  `  96  */
    {0x20, 0x54, 0x54, 0x54, 0x78},  /*  a  97  */
    {0x7F, 0x50, 0x48, 0x48, 0x30},  /*  b  98  */
    {0x38, 0x44, 0x44, 0x44, 0x20},  /*  c  99  */
    {0x38, 0x44, 0x44, 0x48, 0x7F},  /*  d  100 */
    {0x38, 0x54, 0x54, 0x54, 0x18},  /*  e  101 */
    {0x08, 0x7E, 0x09, 0x01, 0x02},  /*  f  102 */
    {0x0C, 0x52, 0x52, 0x52, 0x3E},  /*  g  103 */
    {0x7F, 0x08, 0x04, 0x04, 0x78},  /*  h  104 */
    {0x00, 0x44, 0x7D, 0x40, 0x00},  /*  i  105 */
    {0x20, 0x40, 0x44, 0x3D, 0x00},  /*  j  106 */
    {0x7F, 0x10, 0x28, 0x44, 0x00},  /*  k  107 */
    {0x00, 0x41, 0x7F, 0x40, 0x00},  /*  l  108 */
    {0x78, 0x04, 0x18, 0x04, 0x78},  /*  m  109 */
    {0x7C, 0x08, 0x04, 0x04, 0x78},  /*  n  110 */
    {0x38, 0x44, 0x44, 0x44, 0x38},  /*  o  111 */
    {0x7C, 0x14, 0x14, 0x14, 0x08},  /*  p  112 */
    {0x08, 0x14, 0x14, 0x18, 0x7C},  /*  q  113 */
    {0x7C, 0x08, 0x04, 0x04, 0x08},  /*  r  114 */
    {0x48, 0x54, 0x54, 0x54, 0x20},  /*  s  115 */
    {0x04, 0x3F, 0x44, 0x40, 0x20},  /*  t  116 */
    {0x3C, 0x40, 0x40, 0x20, 0x7C},  /*  u  117 */
    {0x1C, 0x20, 0x40, 0x20, 0x1C},  /*  v  118 */
    {0x3C, 0x40, 0x30, 0x40, 0x3C},  /*  w  119 */
    {0x44, 0x28, 0x10, 0x28, 0x44},  /*  x  120 */
    {0x0C, 0x50, 0x50, 0x50, 0x3C},  /*  y  121 */
    {0x44, 0x64, 0x54, 0x4C, 0x44},  /*  z  122 */
    {0x00, 0x08, 0x36, 0x41, 0x00},  /*  {  123 */
    {0x00, 0x00, 0x7F, 0x00, 0x00},  /*  |  124 */
    {0x00, 0x41, 0x36, 0x08, 0x00},  /*  }  125 */
    {0x0C, 0x02, 0x0C, 0x10, 0x0C},  /*  ~  126 */
    {0x00, 0x00, 0x00, 0x00, 0x00}  /*     127 */
};
