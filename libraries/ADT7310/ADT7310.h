
#ifndef ADT7310_H
#define ADT7310_H

#include "mbed.h"

/**
 * @brief Analog Devices ADT7310 temperature sensor
 */
class ADT7310
{
public:


#define ADT7310_READ (1<<6)
#define ADT7310_WRITE (0)
#define ADT7310_DUMMY (0)
#define ADT7310_STATUS (0)
#define ADT7310_CONFIG (1)
#define ADT7310_TEMP  (2)
#define ADT7310_ID    (3)
#define ADT7310_TCRIT (4)
#define ADT7310_THYST (5)
#define ADT7310_THIGH (6)
#define ADT7310_TLOW  (7)


	/**
	 * @brief ADT7310 class
	 * @param CS - chipselect pin
	 * @param MOSI - MOSI pin
	 * @param MISO - MISO pin
	 * @param SCK - Clock pin
	 */
    ADT7310(PinName CS = SPI_CS, PinName MOSI = SPI_MOSI, PinName MISO = SPI_MISO, PinName SCK = SPI_SCK);

    /**
     * @brief resets the ADT7310
     */
    void reset();

    /**
     * @brief reads status register of the temperature sensor
     * @return value of the status register
     */
    uint8_t read_status();

    /**
     * @brief writes configuration register of the temperature sensor
     * @param data - data to be written
     */
    void write_config(uint8_t data);

    /**
     * @brief issues a conversion to the temperature sensor
     */
    void start_single_conversion();

    /**
     * @brief reads configuration register
     * @return configuration register value
     */
    uint8_t read_config();

    /**
     * @brief reads the temperature
     * @return temperature
     */
    uint16_t read_temp();

    /**
     * @brief writes temperature setpoints
     * @param setpoint - setpoint register
     * @param data - data to be written to the setpoint register
     */
    void write_temp_setpoint(uint8_t setpoint, uint16_t data);

    void spi_write(uint8_t *data, uint8_t size);
    void spi_read(uint8_t *data, uint8_t size);
    //DigitalIn miso;
    DigitalOut cs;
    SPI adt7310;
private:
    const uint8_t SPI_MODE = 0x03;
};

#endif
