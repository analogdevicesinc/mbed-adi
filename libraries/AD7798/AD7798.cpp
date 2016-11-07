/***************************************************************************//**
 *   @file   AD7798.c
 *   @brief  Implementation of AD7798 Driver.
 *   @author
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
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
********************************************************************************
 *   SVN Revision: 577
*******************************************************************************/

/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
#include "AD7798.h"				// AD7798 definitions.
#include <stdio.h>


/***************************************************************************//**
 * @brief Initializes the AD7798 and checks if the device is present.
 *
 * @param None.
 *
 * @return status - Result of the initialization procedure.
 *                  Example: 1 - if initialization was successful (ID is 0x0B).
 *                           0 - if initialization was unsuccessful.
*******************************************************************************/
AD7798::AD7798( PinName CS, PinName MOSI, PinName MISO, PinName SCK):
    /*miso(MISO),*/ ad7798(MOSI, MISO, SCK), cs(CS)
{
    cs = true; // cs is active low
    ad7798.format(8, _SPI_MODE);
}
uint8_t AD7798::init(void)
{
    unsigned char status = 0x1;

    if ((get_register_value (AD7798_REG_ID, 1) & 0x0F) != AD7798_ID) {
        status = 0x0;
    }

    return(status);
}

/***************************************************************************//**
 * @brief Sends 32 consecutive 1's on SPI in order to reset the part.
 *
 * @param None.
 *
 * @return  None.
*******************************************************************************/
void AD7798::frequency(int hz)
{
    ad7798.frequency(hz);
}
void AD7798::reset(void)
{
    unsigned char dataToSend[4] = {0xFF, 0xFF, 0xFF, 0xFF};

    spi_write(dataToSend, 4);

    wait_ms(500);

}
/***************************************************************************//**
 * @brief Reads the value of the selected register
 *
 * @param regAddress - The address of the register to read.
 * @param size - The size of the register to read.
 *
 * @return data - The value of the selected register register.
*******************************************************************************/
uint16_t AD7798::get_register_value(uint8_t regAddress, uint8_t size, bool toggle_cs)
{
    unsigned char data[size];
    uint16_t receivedData = 0x00;
    unsigned char byte;

    data[0] = AD7798_COMM_READ |  AD7798_COMM_ADDR(regAddress);

    spi_read(data, size, toggle_cs);

    receivedData = data[0];

    if(size > 1) {

        for(byte = 1; byte < size; byte++) {
            receivedData = (receivedData << (byte * 8) | data[byte]);
        }
    }
    return receivedData;
}
/***************************************************************************//**
 * @brief Writes the value to the register
 *
 * @param -  regAddress - The address of the register to write to.
 * @param -  regValue - The value to write to the register.
 * @param -  size - The size of the register to write.
 *
 * @return  None.
*******************************************************************************/
void AD7798::set_register_value(uint8_t regAddress, uint16_t regValue,
                                uint8_t size, bool toggle_cs)
{
    uint8_t data[size + 1];
    uint8_t byte;
    uint16_t mask;

    data[0] = AD7798_COMM_WRITE |  AD7798_COMM_ADDR(regAddress);

    if(size == 1) {

        mask = 0x00FF;

    } else {

        mask = 0xFF00;
    }

    for(byte = 1; byte <= size; byte++) {
        data[byte] = (uint8_t)((regValue & mask) >> ((size - byte) * 8));
        mask = mask >> (byte * 8);
    }

    spi_write(data, (1 + size), toggle_cs);


}
/***************************************************************************//**
 * @brief Reads /RDY bit of status reg.
 *
 * @param None.
 *
 * @return rdy	- 0 if RDY is 1.
 *              - 1 if RDY is 0.
*******************************************************************************/
uint8_t AD7798::ready(void)
{

    while((get_register_value( AD7798_REG_STAT, 1) & 0x80) != 0x80);

    return(1);
}

/***************************************************************************//**
 * @brief Sets the operating mode of AD7798.
 *
 * @param mode - Mode of operation.
 *
 * @return  None.
*******************************************************************************/
void AD7798::set_mode(uint8_t mode)
{
    unsigned long command;
    command = get_register_value(AD7798_REG_MODE, 2);
    command &= ~AD7798_MODE_SEL(0xFF);
    command |= AD7798_MODE_SEL(mode);
    set_register_value(
        AD7798_REG_MODE,
        command,
        2
    );
}
/***************************************************************************//**
 * @brief Selects the channel of AD7798.
 *
 * @param  channel - ADC channel selection.
 *
 * @return  None.
*******************************************************************************/
void AD7798::set_channel(uint8_t channel)
{
    unsigned long command;
    command = get_register_value(AD7798_REG_CONF, 2);
    command &= ~AD7798_CONF_CHAN(0xFF);
    command |= AD7798_CONF_CHAN(channel);
    set_register_value(
        AD7798_REG_CONF,
        command,
        2
    );
}

/***************************************************************************//**
 * @brief  Sets the gain of the In-Amp.
 *
 * @param  gain - Gain.
 *
 * @return  None.
*******************************************************************************/
void AD7798::set_gain(uint16_t gain)
{
    uint16_t command;
    command = get_register_value(AD7798_REG_CONF, 2);
    command &= ~AD7798_CONF_GAIN(0xFF);
    command |= AD7798_CONF_GAIN(gain);
    set_register_value(
        AD7798_REG_CONF,
        command,
        2
    );
}

void AD7798::set_filter(uint8_t filter)
{
    unsigned long command;
    command = get_register_value(AD7798_REG_MODE, 2);
    command &= ~AD7798_MODE_RATE(0x0F);
    command |= AD7798_MODE_RATE(filter);
    set_register_value(
        AD7798_REG_MODE,
        command,
        2
    );
}
/***************************************************************************//**
 * @brief Enables or disables the reference detect function.
 *
 * @param state - State of the reference detect function.
 *               Example: 0	- Reference detect disabled.
 *                        1	- Reference detect enabled.
 *
 * @return None.
*******************************************************************************/
void AD7798::set_reference(uint8_t state)
{
    unsigned long command = 0;
    command = get_register_value(AD7798_REG_CONF, 2);
    command &= ~AD7798_CONF_REFDET(1);
    command |= AD7798_CONF_REFDET(state);
    set_register_value(AD7798_REG_CONF,
                       command,
                       2);
}

void AD7798::set_coding_mode(uint8_t mode)
{
    uint16_t command;

    command = get_register_value(AD7798_REG_CONF, 2);

    if(mode == AD7798_BIPOLAR) {

        command &= ~AD7798_CONF_UNIPOLAR;

    } else if(mode == AD7798_UNIPOLAR) {

        command |= AD7798_CONF_UNIPOLAR;
    }
    set_register_value(
        AD7798_REG_CONF,
        command,
        2
    );
}

void AD7798::set_burnout_current(uint8_t select)
{
    uint16_t command;

    command = get_register_value(AD7798_REG_CONF, 2);

    if(select == AD7798_DISABLE)
        command &= ~AD7798_CONF_BO_EN;
    else if(select == AD7798_ENABLE)
        command |= AD7798_CONF_BO_EN;

    set_register_value(
        AD7798_REG_CONF,
        command,
        2
    );
}

uint8_t AD7798::spi_read(uint8_t *data, uint8_t bytes_number, bool toggle_cs)
{
    cs = false & toggle_cs;
    data[0] = ad7798.write(data[0]);
    for(uint8_t byte = 1; byte <= bytes_number; byte++) {
        data[byte - 1] = ad7798.write(data[byte]);
    }
    cs = true & toggle_cs;
    return bytes_number;
}
uint8_t AD7798::spi_write(uint8_t *data, uint8_t bytes_number, bool toggle_cs)
{
    cs = false & toggle_cs;
    for(uint8_t byte = 0; byte < bytes_number; byte++) {
        ad7798.write(data[byte]);
    }
    cs = true & toggle_cs;
    return bytes_number;
}

void AD7798::read_data(uint8_t adcChannel, uint16_t *adcData)
{

    uint8_t channel;

    channel = 0x80 | adcChannel;

    cs = 0;

    set_register_value(AD7798_REG_MODE, 0x200A, 2);//, false);
    uint16_t regVal = 0;
    while( (regVal & channel) != channel) {
        regVal = get_register_value( AD7798_REG_STAT, 1);//, false);
    }

    //timer_sleep(200);
    wait_ms(200); // ???

    *adcData = get_register_value(AD7798_REG_DATA, 2);//, false);

    cs = 1;



}
