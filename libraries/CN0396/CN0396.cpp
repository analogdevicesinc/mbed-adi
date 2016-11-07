
#include "AD5270.h"
#include <math.h>
#include "CN0396.h"

#define ADC_GAIN      AD7798_GAIN_1
#define ADC_SPS        0x05  //50SPS

#define CO_SENS    (75 * pow(10, -9))    /* Sensitivity nA/ppm in 400ppm CO 50 to 100 */
#define CO_RANGE   1000 /* Range ppm CO limit of performance warranty 1,000 */
#define H2S_SENS   (700 * pow(10, -9)) /* Sensitivity nA/ppm in 20ppm H2S 450 to 900 */
#define H2S_RANGE  100  /* Range ppm H2S limit of performance warranty 100 */

/*	CO side	H2S side
Temperature	Mean 	Mean*/

extern Serial pc;

CN0396::CN0396(PinName csad, PinName csrdac, PinName cstemp) :
    csad(csad), csrdac(csrdac), cstemp(cstemp), ad(csad), rdac(csrdac), temp(cstemp)
{

}

void CN0396::data_to_voltage(uint16_t adcValue, float *voltage, int gain_adc)
{
    *voltage = (float)(adcValue * V_REF) / (float)(_2_16 * gain_adc);
}

void CN0396::data_to_voltage_bipolar(uint16_t adcValue, float *voltage, int gain_adc)
{
    *voltage = ((static_cast<float>(adcValue) / _2_15) - 1.0) * (V_REF / static_cast<float>(gain_adc));
}

float CN0396::get_feedback_resistor_value(float sensitivity, float range)
{
    return 1.2 / (sensitivity * range);
}

void CN0396::configure_feedback_resistors(float resistance1, float resistance2)
{
    uint16_t R1 = rdac.calc_RDAC(resistance1);
    uint16_t R2 = rdac.calc_RDAC(resistance2);

    csrdac = false;
    rdac.write_cmd(AD5270::WRITE_CTRL_REG,  AD5270::RDAC_WRITE_PROTECT, false); // RDAC register write protect -  allow update of wiper position through digital interface
    rdac.write_cmd(AD5270::WRITE_CTRL_REG,  AD5270::RDAC_WRITE_PROTECT, false); // RDAC register write protect -  allow update of wiper position through digital interface
    csrdac = true;
    wait_us(2);
    csrdac = false;
    rdac.write_cmd(AD5270::WRITE_RDAC, R2, false); // write data to the RDAC register
    rdac.write_cmd(AD5270::WRITE_RDAC, R1, false); // write data to the RDAC register
    csrdac = true;
    wait_us(2);
    csrdac = false;
    rdac.write_cmd(AD5270::WRITE_CTRL_REG, 0, false); // RDAC register write protect -  allow update of wiper position through digital interface
    rdac.write_cmd(AD5270::WRITE_CTRL_REG, 0, false); // RDAC register write protect -  allow update of wiper position through digital interface
    csrdac = false;
    wait_us(2);
    csrdac = false;
    rdac.write_reg(AD5270::HI_Z_Cmd, false);
    rdac.write_reg(AD5270::HI_Z_Cmd, false);
    csrdac = true;
    wait_us(2);
    csrdac = false;
    rdac.write_reg(AD5270::NO_OP_cmd, false);
    rdac.write_reg(AD5270::NO_OP_cmd, false);
    csrdac = true;
}

void CN0396::init()
{
    // set rdac

    pc.printf("Computing resistor values \r\n");

    resistance1 = get_feedback_resistor_value(CO_SENS,  CO_RANGE );
    resistance0 = get_feedback_resistor_value(H2S_SENS, H2S_RANGE);

    pc.printf("R1 = %f\r\nR2=%f\r\n", resistance0, resistance1);
    pc.printf("Configuring feedback resistors\r\n");
    configure_feedback_resistors(resistance1, resistance1);
    pc.printf("Done\r\n");
    // config temp
    pc.printf("Configuring temperature sensor\r\n");
    temp.reset();
    temp.write_config(0x90);
    pc.printf("Done\r\n");

    pc.printf("Configuring ADC\r\n");
    ad.reset();
    if(ad.init()) {
        ad.set_coding_mode(AD7798_UNIPOLAR);
        ad.set_mode(AD7798_MODE_SINGLE);
        ad.set_gain(ADC_GAIN);
        ad.set_filter(ADC_SPS);
        ad.set_reference(AD7798_REFDET_ENA);
        pc.printf("ADC Config succesful\r\n");
    } else {
        pc.printf("ADC Config failed\r\n");

    }


}

float CN0396::compensate_ppm(float result, float temp, sensor_type_t sensor)
{
    for(uint8_t i = 1; i < COMPENSATION_TABLE_SIZE; i++) {
        if(temp < ppm_compensation[i].temp && temp > ppm_compensation[i - 1].temp) {
            float compensation_coef;
            if(sensor == H2S_SENSOR) {
                compensation_coef = (((temp - (ppm_compensation[i - 1].temp )) * (ppm_compensation[i].H2S_percent - ppm_compensation[i - 1].H2S_percent)) / (ppm_compensation[i].temp  - ppm_compensation[i - 1].temp)) + ppm_compensation[i - 1].H2S_percent;
            } else {
                compensation_coef = (((temp - (ppm_compensation[i - 1].temp )) * (ppm_compensation[i].CO_percent - ppm_compensation[i - 1].CO_percent)) / (ppm_compensation[i].temp  - ppm_compensation[i - 1].temp)) + ppm_compensation[i - 1].CO_percent;
            }

            return (result * compensation_coef) / 100.0;
        }
    }
}
void CN0396::read()
{
    uint16_t data0, data1;
    // read temperature
    uint16_t temp_data = temp.read_temp();
    float temp = 0;

    if(temp_data & 0x8000) {
        temp = (temp_data - 65536) / (128.0);
    } else {
        temp = temp_data / (128.0);
    }

    // read channels
    ad.set_channel(0);
    ad.read_data(0, &data0);
    float volt0;
    data_to_voltage(data0, &volt0);
    float result0 = (volt0 / resistance0) / CO_SENS;
    ad.set_channel(1);
    ad.read_data(1, &data1);
    float volt1;
    data_to_voltage(data1, &volt1);
    float result1 = (volt1 / resistance1) / H2S_SENS;
    // compute ppm based on formula
    // return ppm
    result0 = compensate_ppm(result0, temp, CO_SENSOR);
    result1 = compensate_ppm(result1, temp, H2S_SENSOR);

    pc.printf("%f %f %f \r\n", temp, result0, result1);
}
