/**
*   @file     CN0357.cpp
*   @brief    Source file for CN0357 toxic gas sensor shield
*   @version  V0.1
*   @author   ADI
*   @date     March 2015
**/


#include "mbed.h"
#include "AD7790.h"
#include "AD5270.h"
#include "cn0357.h"

/**
 * @brief CN0357 constructor
 * @param CSAD7790 - (optional)chip select of the AD7790
 * @param CSAD5270 - (optional)chip select of the AD5270
 * @param MOSI - (optional)pin of the SPI interface
 * @param MISO - (optional)pin of the SPI interface
 * @param SCK  - (optional)pin of the SPI interface
 */
CN0357::CN0357(PinName CSAD7790, PinName CSAD5270, PinName MOSI, PinName MISO,
               PinName SCK) :
    ad7790(CSAD7790, MOSI, MISO, SCK), ad5270(CSAD5270, 20000.0, MOSI, MISO, SCK) ,
    _sensor_sensitivity(0), _sensor_range(0), _RDACvalue(0)
{
}

/**
 * @brief initializes the AD7790 and the AD5270
 * @param range - range of the sensor used (in ppm)
 * @param sensitivity - sensitivity of the sensor (A/ppm)
 * @param jp - (optional)jumper configuration of the CN0357
 * @param mode_val - (optional)if jp is set to InternalADC, configures the mode register of the Internal ADC
 * @param filter_val - (optional)if jp is set to InternalADC, configures the filter register of the Internal ADC
 */
void CN0357::init(float range, float sensitivity,JumperConfig_t jp,uint8_t mode_val, uint8_t filter_val)
{
    ad5270.frequency(500000);
    ad7790.frequency(500000);

    float resistance = set_sensor_parameters(range,sensitivity);
    _rdac_init(resistance);
    if(jp == INTERNAL_AD7790) {
        _AD7790_init(mode_val, filter_val);
    }

}

/**
 * @brief initializes the RDAC and sets SDO to HiZ
 * @param resistance - resistance value to initialize the RDAC
 * @return
 */
void CN0357::_rdac_init(float resistance)
{
    /* RDAC initialization*/
    /* Compute for the nearest RDAC value from given resistance and save data to the structure */
    set_RDAC_value(resistance);
    /* Set AD5270 SDO to Hi-Z */
    ad5270.set_SDO_HiZ();
}

/**
 * @brief initializes the AD7790
 * @param mode_val -  configures the mode register of the Internal ADC
 * @param filter_val -  configures the filter register of the Internal ADC
 */
void CN0357::_AD7790_init(uint8_t mode_val, uint8_t filter_val)
{
    ad7790.reset();
    // wait_ms(1000);
    ad7790.write_reg(AD7790::MODE_REG, mode_val);
    ad7790.write_reg(AD7790::FILTER_REG, filter_val);

}

/**
 * @brief reads the status register of the AD7790
 * @return status register value
 */
uint8_t CN0357::read_adc_status(void)
{
    return ad7790.read_reg(AD7790::STATUS_REG);
}

/**
 * @brief reads the ADC and computes the sensor voltage
 * @return sensor voltage
 */
float CN0357::read_sensor_voltage(void)
{
    return data_to_voltage(read_sensor());
}

/**
 * @brief reads the data register of the AD7790
 * @return data register value
 */
uint16_t CN0357::read_sensor(void)
{
    return ad7790.read_data();
}

/**
 * @brief reads and computes the sensor reading in PPM
 * @return value of the sensor reading in PPM
 */
float CN0357::read_ppm()
{
    return calc_ppm(read_sensor_voltage()); /* Convert voltage to Gas concentration*/
}

/**
 * @brief computes a value in PPM from a reading received as a param
 * @param adcVoltage - voltage to be converted to PPM
 * @return sensor value in PPM
 */
float CN0357::calc_ppm(float adcVoltage)
{
    float fConcentration = 0;
    fConcentration = (fabs(adcVoltage) / _RDACvalue) / _sensor_sensitivity;
    return fConcentration;
}

/**
 * @brief computes voltage from a 16 bit ADC value received as a parameter
 * @param data - ADC value
 * @return sensor voltage
 */
float CN0357::data_to_voltage(uint16_t data)
{
    return ((static_cast<float>(data) / pow(2, 15)) - 1) * 1.2; /* Bipolar voltage computation from ADC code */
}

/**
 * @brief sets a new value for the RDAC
 * @param resistance new value for the resistance
 * @return none
 */
void CN0357::set_RDAC_value(float resistance)
{
    // Compute for the RDAC code nearest to the required feedback resistance
    uint16_t RDAC_val = (resistance / 20000.0) * 1024.0;
    // Compute for the constants used in voltage and PPM conversion computation
    _RDACvalue = ((static_cast<float> (RDAC_val) * 20000.0) / 1024.0);
    ad5270.write_cmd(AD5270::WRITE_CTRL_REG, 0x02); // RDAC register write protect -  allow update of wiper position through digital interface
    ad5270.write_cmd(AD5270::WRITE_RDAC, RDAC_val); // write data to the RDAC register
}

/**
 * @brief getter method for RDAC value
 * @return value of the RDAC in ohms
 */
float CN0357::get_RDAC_value()
{
    return _RDACvalue;
}

/**
 * @brief set sensor range and sensitivity
 * sets sensor range, sensitivity
 * returns suggested resistance value for feedback resistor
 * @param range - range of the sensor used (in ppm)
 * @param sensitivity - sensitivity of the sensor (in A/ppm)
 * @return suggested resistance value for feedback resistor
 */
float CN0357::set_sensor_parameters(float range, float sensitivity)
{
    _sensor_sensitivity = static_cast<float>(sensitivity);
    _sensor_range = range;
    return (1.2 / (static_cast<float>(_sensor_range * _sensor_sensitivity)));
}

/**
 * @brief getter method for sensor sensitivity
 * @return sensor sensitivity (in A/ppm)
 */
float CN0357::get_sensor_sensitivity()
{
    return _sensor_sensitivity;
}

/**
 * @brief getter method for sensor range
 * @return sensor range (in ppm)
 */
float CN0357::get_sensor_range()
{
    return _sensor_range;
}
