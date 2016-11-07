#ifndef _CN0396_H_
#define _CN0396_H_
#include <mbed.h>
#include "AD7798.h"
#include "AD5270.h"
#include "ADT7310.h"

/**
 * @brief The CN0396 class
 */
class CN0396
{
public:

#define V_REF                1.200    // [V]
#define _2_16                65535.0   // 2^16
#define _2_15                32767.0   // 2^16
#define COMPENSATION_TABLE_SIZE 9

    typedef enum {
        CO_SENSOR,
        H2S_SENSOR
    } sensor_type_t;

    typedef struct {
        int8_t temp;
        float CO_percent;
        float H2S_percent;
    } ppm_compensation_t;

	/**
	 *  @brief compensation look-up table
	 */
    const ppm_compensation_t ppm_compensation[COMPENSATION_TABLE_SIZE] = {
        { -30	, 29.9  , 82.3 },
        { -20	, 38.8  , 84.6 },
        { -10	, 53.7  , 88.6 },
        {0		, 69.6  , 92.2 },
        {10		, 84.9  , 96.2 },
        {20		, 100.0 , 100.0},
        {30		, 112.7 , 103.1},
        {40		, 123.7 , 105.6},
        {50		, 133.1 , 107.4}
    };


    /**
     * @brief CN0396 class constructor
     * @param csad  - chipselect pin of the ADC
     * @param csrdac - chipselect pin of the RDAC
     * @param cstemp - chipselect pin of the temperature sensor
     */
    CN0396(PinName csad, PinName csrdac, PinName cstemp);
    /**
     * @brief Initializes the board
     */
    void init();

    /**
     * @brief - Reads the sensor and computes the PPM values
     */
    void read();

    /**
     * @brief computes the feedback resistor value for the sensor
     * @param sensitivity - sensor sensitivity
     * @param range - sensor range
     * @return resistor value
     */
    float get_feedback_resistor_value(float sensitivity, float range);

    /**
     * @brief configures the RDACs with the resistance values
     * @param resistance1 - resistance of RDAC1
     * @param resistance2 - resistance of RDAC2
     * @return
     */
    void configure_feedback_resistors(float resistance1, float resistance2);

    /**
    * @brief computes ADC counts-to-voltage in unipolar configuration
    * @param adcValue - value in counts
    * @param voltage - voltage value returned by the method
    * @param gain_adc - the gain of the adc
    */
    void data_to_voltage(uint16_t adcValue, float *voltage, int gain_adc = 1);

    /**
     * @brief computes ADC counts-to-voltage in bipolar configuration
     * @param adcValue - value in counts
     * @param voltage - voltage value returned by the method
     * @param gain_adc - the gain of the adc
     */
    void data_to_voltage_bipolar(uint16_t adcValue, float *voltage, int gain_adc = 1);

    /**
     * @brief compensates ppm value based on temperature reading
     * @param result - ppm value before compensation
     * @param temp - temperature used in compensation
     * @param sensor - sensor id
     * @return compensated value
     */
    float compensate_ppm(float result, float temp, sensor_type_t sensor);
    DigitalOut csad, csrdac, cstemp;
    AD7798 ad;
    AD5270 rdac;
    ADT7310 temp;
    float resistance0, resistance1;
private:


};
#endif
