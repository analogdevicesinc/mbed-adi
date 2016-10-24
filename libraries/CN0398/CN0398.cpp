#include "CN0398.h"
#include "AD7124.h"
#include <mbed.h>

#define RREF (5000.0)
#define TEMP_GAIN (16.0)
#define PT100_RESISTANCE_TO_TEMP(x) ((x-100.0)/(0.385))
#define _2_23 (1<<23)

#define CALIBRATION_NR_OF_SAMPLES (5)

extern Serial pc;

#define ms_delay (1)

CN0398::CN0398(PinName cs, PinName adp7118enable) : ad7124(cs), ADP7118Enable(adp7118enable), offset_voltage(default_offset_voltage)
{
    calibration_ph[0][0] = default_calibration_ph[0][0];
    calibration_ph[0][1] = default_calibration_ph[0][1];
    calibration_ph[1][0] = default_calibration_ph[1][0];
    calibration_ph[1][1] = default_calibration_ph[1][1];
    solution0 = 0;
    solution1 = 0;
}

void CN0398::calibrate_ph_pt0(float temperature)
{
    float volt = 0;
    for(int i = 0; i < CALIBRATION_NR_OF_SAMPLES; i++) {
        set_digital_output(P2, true);
        int32_t data = read_channel(0);
        set_digital_output(P2, false);
        volt += data_to_voltage_bipolar(data >> 8, 1, 3.3);
    }
    volt = volt / CALIBRATION_NR_OF_SAMPLES;
    if(temperature < 0) {
        calibration_ph[0][0] = ph_temp_lut[solution0][0];
    } else {
        for(uint8_t i = 1; i < NUMBER_OF_TEMPERATURE_ENTRIES; i++) {
            if(temperature > ph_temperatures[i - 1] && temperature <= ph_temperatures[i]) {
                calibration_ph[0][0] = ph_temp_lut[solution0][i];
                break;
            }
        }
    }
    calibration_ph[0][1] = volt;
    pc.printf("Calibration solution 1 ph: %f with sensor voltage of %f\r\n", calibration_ph[0][0], volt);
}
void CN0398::calibrate_ph_pt1(float temperature)
{
    float volt = 0;
    for(int i = 0; i < CALIBRATION_NR_OF_SAMPLES; i++) {
        set_digital_output(P2, true);
        int32_t data = read_channel(0);
        set_digital_output(P2, false);
        volt += data_to_voltage_bipolar(data >> 8, 1, 3.3);
    }

    volt = volt / CALIBRATION_NR_OF_SAMPLES;
    if(temperature < 0) {
        calibration_ph[1][0] = ph_temp_lut[solution1][0];
    } else {
        for(uint8_t i = 1; i < NUMBER_OF_TEMPERATURE_ENTRIES; i++) {
            if(temperature > ph_temperatures[i - 1] && temperature <= ph_temperatures[i]) {
                calibration_ph[1][0] = ph_temp_lut[solution1][i];
                break;
            }
        }
    }
    calibration_ph[1][1] = volt;
    pc.printf("Calibration solution 2 ph: %f with sensor voltage of %f\r\n", calibration_ph[1][0], volt);
}

void CN0398::calibrate_ph_offset()
{
    float volt = 0;
    for(int i = 0; i < CALIBRATION_NR_OF_SAMPLES; i++) {
        set_digital_output(P2, true);
        int32_t data = read_channel(0);
        set_digital_output(P2, false);
        volt += data_to_voltage_bipolar(data >> 8, 1, 3.3);
    }
    offset_voltage = volt / CALIBRATION_NR_OF_SAMPLES;
    pc.printf("Offset voltage is: %f \r\n", volt);
}


float CN0398::read_rtd()
{
    float temperature = 25.0;
#ifdef TEMPERATURE_SENSOR_PRESENT
    int32_t data = read_channel(2);
    data = (data >> 8) & 0x00ffffff;
    float resistance = ((static_cast<float>(data) - _2_23) * RREF) / (TEMP_GAIN * _2_23);
#ifdef USE_LINEAR_TEMP_EQ
    temperature = PT100_RESISTANCE_TO_TEMP(resistance);
#else

#define A (3.9083*pow(10,-3))
#define B (-5.775*pow(10,-7))
    /*if(resistance < 100.0)
        temperature = -242.02 + 2.228 * resistance + (2.5859 * pow(10, -3)) * pow(resistance, 2) - (48260 * pow(10, -6)) * pow(resistance, 3) - (2.8183 * pow(10, -3)) * pow(resistance, 4) + (1.5243 * pow(10, -10)) * pow(resistance, 5);
    else*/
    temperature = ((-A + sqrt(double(pow(A, 2) - 4 * B * (1 - resistance / 100.0))) ) / (2 * B));
#endif
#endif
    return temperature;

}

int32_t CN0398::read_channel(uint8_t ch)
{
    int32_t data;
    enable_channel(ch);
    start_single_conversion();

    if (ad7124.WaitForConvReady(10000) == -3) {
        pc.printf("TIMEOUT");
        return -1;
    }
    ad7124.ReadData(&data);
    disable_channel(ch);
    return data;

}
float CN0398::read_ph(float temperature)
{
    float ph = 0;
#ifdef PH_SENSOR_PRESENT
    set_digital_output(P2, true);
    int32_t data = read_channel(0);
    set_digital_output(P2, false);
    float volt = data_to_voltage_bipolar(data >> 8, 1, 3.3);
#ifdef DEBUG_MODE
    pc.printf("pH sensor voltage - %f\n", volt);
#endif

    if(use_nernst) {
        ph  = -((volt - ZERO_POINT_TOLERANCE) / ((2.303 * AVOGADRO * (temperature + KELVIN_OFFSET)) / FARADAY_CONSTANT) ) + PH_ISO;
    } else {
        float m =  (calibration_ph[1][0] - calibration_ph[0][0]) / (calibration_ph[1][1] - calibration_ph[0][1]);
        ph = m * (volt - calibration_ph[1][1] + offset_voltage) + calibration_ph[1][0];
    }
#endif
    return ph;
}
float CN0398::read_moist()
{
    float moisture = 0;
#ifdef MOISTURE_SENSOR_PRESENT
    ADP7118Enable = true;
    set_digital_output(P3, true);
    wait_ms(SENSOR_SETTLING_TIME);
    int32_t data = read_channel(1);
    ADP7118Enable = false;
    set_digital_output(P3, false);

    data = (data >> 8) & 0x00ffffff;
    float volt = data_to_voltage(data, 1, 3.3);
#ifdef USE_MANUFACTURER_MOISTURE_EQ
    if(volt <= 1.1) {
        moisture = 10 * volt - 1;
    } else if(volt > 1.1 && volt <= 1.3) {
        moisture = 25 * volt - 17.5;
    } else if(volt > 1.3 && volt <= 1.82) {
        moisture = 48.08 * volt - 47.5;
    } else if(volt > 1.82) {
        moisture = 26.32 * volt - 7.89;
    }
#else
    moisture = -1.18467 + 21.5371 * volt - 110.996 * (pow(volt, 2)) + 397.025 * (pow(volt, 3)) - 666.986 * (pow(volt, 4)) + 569.236 * (pow(volt, 5)) - 246.005 * (pow(volt, 6)) + 49.4867 * (pow(volt, 7)) - 3.37077 * (pow(volt, 8));
#endif
    if(moisture > 100) moisture = 100;
    if(moisture < 0 ) moisture = 0;
#endif
    return moisture;
}

float CN0398::data_to_voltage_bipolar(uint32_t data, uint8_t gain, float VREF)
{
    data = data & 0xFFFFFF;
    return ((data / static_cast<float>(0xFFFFFF / 2)) - 1) * (VREF / gain);
}

float CN0398::data_to_voltage(uint32_t data, uint8_t gain, float VREF)
{
    data = data & 0xFFFFFF;
    return (data / static_cast<float>(0xFFFFFF)) * (VREF / gain);
}

void CN0398::enable_channel(int channel)
{
    AD7124::ad7124_registers regNr = static_cast<AD7124::ad7124_registers> (AD7124::AD7124_Channel_0 + channel); //Select ADC_Control register
    uint32_t setValue = ad7124.ReadDeviceRegister(regNr);
    setValue |= (uint32_t) AD7124_CH_MAP_REG_CH_ENABLE;  //Enable channel0
    setValue &= 0xFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);    // Write data to ADC
    wait_ms(ms_delay);
}

void CN0398::disable_channel(int channel)
{
    AD7124::ad7124_registers regNr = static_cast<AD7124::ad7124_registers> (AD7124::AD7124_Channel_0 + channel); //Select ADC_Control register
    uint32_t setValue = ad7124.ReadDeviceRegister(regNr);
    setValue &= (~(uint32_t) AD7124_CH_MAP_REG_CH_ENABLE);  //Enable channel0
    setValue &= 0xFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);    // Write data to ADC
    wait_ms(ms_delay);
}

/*
void CN0398::enable_current_source0(int current_source_channel)
{
    AD7124::ad7124_registers regNr = AD7124::AD7124_IOCon1; //Select ADC_Control register
    uint32_t setValue = ad7124.ReadDeviceRegister(regNr);
    setValue &= ~(AD7124_IO_CTRL1_REG_IOUT_CH0(0xF));
    setValue |= AD7124_IO_CTRL1_REG_IOUT_CH0(current_source_channel);// set IOUT0 current to 500uA
    setValue &= 0xFFFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);    // Write data to ADC
    wait_ms(ms_delay);
}

void CN0398::enable_current_source1(int current_source_channel)
{
    AD7124::ad7124_registers regNr = AD7124::AD7124_IOCon1; //Select ADC_Control register
    uint32_t setValue = ad7124.ReadDeviceRegister(regNr);
    setValue &= ~(AD7124_IO_CTRL1_REG_IOUT_CH1(0xF));
    setValue |= AD7124_IO_CTRL1_REG_IOUT_CH1(current_source_channel);// set IOUT0 current to 500uA
    setValue &= 0xFFFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);    // Write data to ADC
    wait_ms(ms_delay);
}*/

void CN0398::set_digital_output(ad_digital_output_t p, bool state)
{
    AD7124::ad7124_registers regNr = AD7124::AD7124_IOCon1; //Select ADC_Control register
    uint32_t setValue = ad7124.ReadDeviceRegister(regNr);
    if(state)
        setValue |= ((AD7124_8_IO_CTRL1_REG_GPIO_DAT1) << p);
    else
        setValue &= (~((AD7124_8_IO_CTRL1_REG_GPIO_DAT1) << p));
    ad7124.WriteDeviceRegister(regNr, setValue);    // Write data to ADC
    wait_ms(ms_delay);
}


void CN0398::start_single_conversion()
{
    AD7124::ad7124_registers regNr = AD7124::AD7124_ADC_Control; //Select ADC_Control register
    uint32_t setValue = ad7124.ReadDeviceRegister(regNr);
    setValue &= 0xFFC3;
    setValue |= 0x04;               //single conversion;
    setValue |= AD7124_ADC_CTRL_REG_DATA_STATUS;
    setValue &= 0xFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);    // Write data to ADC*/
    wait_ms(ms_delay * 10);
}

void CN0398::reset()
{
    ad7124.frequency(500000);
    ad7124.Reset();
    pc.printf("Reseted AD7124\r\n");
    wait_ms(1000);
}

void CN0398::setup()
{
    ad7124.Setup();
}

void CN0398::init()
{
    uint32_t setValue;
    enum AD7124::ad7124_registers regNr;

    /* Set Config_0 0x19*/
    regNr = AD7124::AD7124_Config_0;               //Select Config_0 register - pH
    setValue = 0;//ad7124.ReadDeviceRegister(regNr);
    setValue |= AD7124_CFG_REG_BIPOLAR;     //Select bipolar operation
    setValue |= AD7124_CFG_REG_BURNOUT(0);  //Burnout current source off
    setValue |= AD7124_CFG_REG_REF_BUFP;
    setValue |= AD7124_CFG_REG_REF_BUFM;
    setValue |= AD7124_CFG_REG_AIN_BUFP;    //Buffer AIN5
    setValue |= AD7124_CFG_REG_AINN_BUFM;   //Buffer AIN4
    setValue |= AD7124_CFG_REG_REF_SEL(0); //REFIN1(+)/REFIN1(−).
    setValue |= AD7124_CFG_REG_PGA(0);
    setValue &= 0xFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);   // Write data to ADC

    /* Set Config_0 0x1A*/
    regNr = AD7124::AD7124_Config_1;               //Select Config_1 register - Moisture
    setValue = 0;//ad7124.ReadDeviceRegister(regNr);
    setValue &= ~AD7124_CFG_REG_BIPOLAR;     //Select bipolar operation
    setValue |= AD7124_CFG_REG_BURNOUT(0);  //Burnout current source off
    setValue |= AD7124_CFG_REG_REF_BUFP;
    setValue |= AD7124_CFG_REG_REF_BUFM;
    setValue |= AD7124_CFG_REG_AIN_BUFP;    //Buffer AIN5
    setValue |= AD7124_CFG_REG_AINN_BUFM;   //Buffer AIN4*/
    setValue |= AD7124_CFG_REG_REF_SEL(0); // REFIN1(+)/REFIN1(−).
    setValue |= AD7124_CFG_REG_PGA(0);
    setValue &= 0xFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);   // Write data to ADC

    /* Set Config_0 0x1B*/
    regNr = AD7124::AD7124_Config_2;               //Select Config_2 register - temp
    setValue = 0;//ad7124.ReadDeviceRegister(regNr);
    setValue |= AD7124_CFG_REG_BIPOLAR;     //Select bipolar operation
    setValue |= AD7124_CFG_REG_BURNOUT(0);  //Burnout current source off
    setValue |= AD7124_CFG_REG_REF_BUFP;
    setValue |= AD7124_CFG_REG_REF_BUFM;
    setValue |= AD7124_CFG_REG_AIN_BUFP;    //Buffer AIN5
    setValue |= AD7124_CFG_REG_AINN_BUFM;   //Buffer AIN4
    setValue |= AD7124_CFG_REG_REF_SEL(1); //REFIN2(+)/REFIN2(-).
    setValue |= AD7124_CFG_REG_PGA(4); // gain 16
    setValue &= 0xFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);   // Write data to ADC

    /* Set Channel_0 register 0x09*/
    regNr = AD7124::AD7124_Channel_0;  // pH reading
    setValue = 0;
    setValue |= AD7124_CH_MAP_REG_SETUP(0);             // Select setup0
    setValue |= AD7124_CH_MAP_REG_AINP(6);         // Set AIN4 as positive input
    setValue |= AD7124_CH_MAP_REG_AINM(7);         // Set AIN5 as negative input
    setValue &= 0xFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);   // Write data to ADC

    regNr = AD7124::AD7124_Channel_1; // Moisture
    setValue = 0;
    setValue |= AD7124_CH_MAP_REG_SETUP(1);             // Select setup0
    setValue |= AD7124_CH_MAP_REG_AINP(8);         // Set AIN4 as positive input
    setValue |= AD7124_CH_MAP_REG_AINM(19);         // Set AIN5 as negative input
    setValue &= 0xFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);   // Write data to ADC

    regNr = AD7124::AD7124_Channel_2; // RTD - gain 16
    setValue = 0;
    setValue |= AD7124_CH_MAP_REG_SETUP(2);             // Select setup0
    setValue |= AD7124_CH_MAP_REG_AINP(9);         // Set AIN4 as positive input
    setValue |= AD7124_CH_MAP_REG_AINM(10);         // Set AIN5 as negative input
    setValue &= 0xFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);   // Write data to ADC

    /* Set IO_Control_1 0x03 */
    regNr = AD7124::AD7124_IOCon1;               //Select IO_Control_1 register
    //setValue = ad7124.ReadDeviceRegister(regNr);
    setValue = 0;
    setValue |= AD7124_8_IO_CTRL1_REG_GPIO_CTRL2; // enable AIN3 as digital output
    setValue |= AD7124_8_IO_CTRL1_REG_GPIO_CTRL3; // enable AIN4 as digital output
    setValue |= AD7124_IO_CTRL1_REG_IOUT_CH0(11); // source ain11
    setValue |= AD7124_IO_CTRL1_REG_IOUT_CH1(12); // source ain12
    setValue |= AD7124_IO_CTRL1_REG_IOUT0(0x4);// set IOUT0 current to 500uA
    setValue |= AD7124_IO_CTRL1_REG_IOUT1(0x4);// set IOUT0 current to 500uA*/
    setValue &= 0xFFFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);// Write data to ADC

    // Set IO_Control_2
    regNr = AD7124::AD7124_IOCon2;               //Select IO_Control_2 register
    setValue = 0;
    setValue |= AD7124_8_IO_CTRL2_REG_GPIO_VBIAS7; // enable AIN3 as digital output
    setValue &= 0xFFFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);// Write data to ADC


    /* Set ADC_Control 0x01 */
    regNr = AD7124::AD7124_ADC_Control;            //Select ADC_Control register
    setValue = ad7124.ReadDeviceRegister(regNr);
    setValue |= AD7124_ADC_CTRL_REG_DATA_STATUS; // set data status bit in order to check on which channel the conversion is
    setValue &= 0xFFC3; // remove prev mode bits
    setValue |= AD7124_ADC_CTRL_REG_MODE(2);
    setValue &= 0xFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);    // Write data to ADC
    wait_ms(ms_delay);
}





