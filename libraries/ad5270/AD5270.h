/**
*   @file     AD5270.h
*   @brief    Header file for AD5270 rheostat
*   @version  V0.1
*   @author   ADI
*   @date     March 2015
**/

#ifndef AD5270_H
#define AD5270_H

#include "mbed.h"

/**
 * @brief Analog Devices AD5270 SPI Digital Rheostat class
 */
class AD5270
{
public:

    /// AD5270 commands
    typedef enum {
        NO_OP               =  0x00,   ///< No data
        NO_OP_cmd           =  0x0000, ///< 16 bit no data
        WRITE_RDAC          =  0x04,   ///< Write to the RDAC Register
        READ_RDAC           =  0x08,   ///< Read from the RDAC Register
        STORE_50TP          =  0x0C,   ///< Write to the RDAC to memory
        SW_RST              =  0x10,   ///< Software reset to last memory location
        READ_50TP_CONTENTS  =  0x14,   ///< Read the last memory contents
        READ_50TP_ADDRESS   =  0x18,   ///< Read the last memory address
        WRITE_CTRL_REG      =  0x1C,   ///< Write to the control Register
        READ_CTRL_REG       =  0x20,   ///< Read from the control Register
        SW_SHUTDOWN         =  0x24,   ///< Software shutdown (0) - Normal, (1) - Shutdown
        HI_Zupper           =  0x80,   ///< Get the SDO line ready for High Z
        HI_Zlower           =  0x01,   ///< Puts AD5270 into High Z mode
        HI_Z_Cmd            =  0x8001   ///< Puts AD5270 into High Z mode*/
    } AD5270Commands_t;

    AD5270(PinName CS=SPI_CS, float max_resistance = 20000.0, PinName MOSI=SPI_MOSI,PinName MISO=SPI_MISO,PinName SCK=SPI_SCK);
    void set_SDO_HiZ(void);
    void frequency(int hz);
    uint16_t calc_RDAC(float resistance);
    uint16_t write_cmd(uint8_t command, uint16_t data = 0x00);
    uint16_t write_reg(uint16_t data);
    float get_max_resistance(void);

    SPI ad5270;    ///< SPI instance of the AD5270
    DigitalOut cs; ///< DigitalOut instance for the chipselect of the AD5270

private:
    const static int _RESET = 0xff;
    const static int _DUMMY_BYTE = 0xAA;
    float _max_resistance;
};

#endif
