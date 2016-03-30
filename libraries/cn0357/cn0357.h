/**
*   @file     CN0357.h
*   @brief    Header file for CN0357 toxic gas sensor shield
*   @version  V0.1
*   @author   ADI
*   @date     March 2015
**/

#ifndef CN0357_H
#define CN0357_H

#include "mbed.h"
#include "ad7790.h"
#include "ad5270.h"

/**
 * @brief EVAL-CN0357 toxic gas sensor shield
 */
class CN0357
{
public:

    /// CN0357 shield jumper configuration
    typedef enum {
        INTERNAL_AD7790 = 0, 	///< The shield's AD7790 is used
        EXTERNAL_ADC			///< Sensor analog output is routed to A1 pin of the shield
    } JumperConfig_t;

    CN0357(PinName CSAD7790=D8, PinName CSAD5270=D6, PinName MOSI=SPI_MOSI,PinName MISO=SPI_MISO,PinName SCK=SPI_SCK);
    void init(float range, float sensitivity,JumperConfig_t jp = INTERNAL_AD7790, uint8_t mode_val = _DEFAULT_MODE_VAL, uint8_t filter_val = _DEFAULT_FILTER_VAL);

    uint8_t  read_adc_status(void);
    uint16_t read_sensor(void);
    float read_sensor_voltage(void);
    float data_to_voltage(uint16_t data);
    float calc_ppm(float adcVoltage);
    float read_ppm(void);

    void  set_RDAC_value(float resistor_val);
    float get_RDAC_value(void);
    float set_sensor_parameters(float range, float sensitivity);
    float get_sensor_range(void);
    float get_sensor_sensitivity(void);


    AD7790 ad7790; ///< AD7790 instance - can be used for manual overriding
    AD5270 ad5270; ///< AD5270 instance - can be used for manual overriding

private:
    const static int _RESET = 0xff;
    const static int _DEFAULT_MODE_VAL = 0x00;
    const static int _DEFAULT_FILTER_VAL = 0x07;
    float _sensor_sensitivity;
    float _sensor_range;
    float _RDACvalue;
    void _rdac_init(float resistanceValue);
    void _AD7790_init(uint8_t mode_val, uint8_t filter_val);


};

#endif // CN0357_H
