#include "ADT7310.h"


ADT7310::ADT7310(PinName CS, PinName MOSI, PinName MISO, PinName SCK) :/* miso(MISO),*/ adt7310(MOSI, MISO, SCK), cs(CS)
{
    cs = true;
    adt7310.format(8, SPI_MODE);
}

void ADT7310::reset()
{
    cs = false;
    adt7310.write(0xff);
    adt7310.write(0xff);
    adt7310.write(0xff);
    adt7310.write(0xff);
    cs = true;
}
uint8_t ADT7310::read_status()
{
    uint8_t spi_data[2] = {ADT7310_READ | (ADT7310_STATUS << 3), ADT7310_DUMMY};
    spi_read(spi_data, 2);
    return spi_data[1];
}

void ADT7310::start_single_conversion()
{
    uint8_t spi_data[2] = {ADT7310_READ | (ADT7310_CONFIG << 3), ADT7310_DUMMY};
    spi_read(spi_data, 2);
    spi_data[0] = ADT7310_WRITE | (ADT7310_CONFIG << 3);
    spi_data[1] = (spi_data[1] & (~(0x60))) |  (0x20); // Oneshot conversion
    spi_write(spi_data, 2);
}
void ADT7310::write_config(uint8_t data)
{
    uint8_t spi_data[2] = {ADT7310_WRITE | (ADT7310_CONFIG << 3), data};
    spi_write(spi_data, 2);
}

uint8_t ADT7310::read_config()
{
    uint8_t spi_data[2] = {ADT7310_READ | (ADT7310_CONFIG << 3), ADT7310_DUMMY};
    spi_read(spi_data, 2);
    return spi_data[1];
}
uint16_t ADT7310::read_temp()
{
    uint8_t spi_data[3] = {ADT7310_READ | (ADT7310_TEMP << 3), ADT7310_DUMMY, ADT7310_DUMMY };
    spi_read(spi_data, 3);
    return ((static_cast<uint16_t>(spi_data[1]) << 8) | spi_data[2]);
}
void ADT7310::write_temp_setpoint(uint8_t setpoint, uint16_t data)
{
    uint8_t spi_data[3] = {ADT7310_WRITE | (setpoint << 3), ((data & 0xff00) >> 8), data & 0xff };
    spi_write(spi_data, 3);
}

void ADT7310::spi_write(uint8_t *data, uint8_t size)
{
    cs = false;
    uint8_t i;
    for(i = 0; i < size; i++)
        adt7310.write(data[i]);
    cs = true;

}
void ADT7310::spi_read(uint8_t *data, uint8_t size)
{
    cs = false;
    uint8_t i;
    for(i = 0; i < size; i++)
        data[i] = adt7310.write(data[i]);

    cs = true;
}
