#include "CN0391.h"
#include "Thermocouple.h"
#include "AD7124.h"

//#define DEBUG_MODE

extern Serial pc;


#define ms_delay (1)
#define R2 (1600.0)
#define RTD_SLOPE (15/39.0)
#define RTD_CONSTANT (1039.0)
#define RTD_CONVERSION(R1) RTD_SLOPE * (R1-RTD_CONSTANT)
#define CAL_CHANNEL (8)
Thermocouple_Channel::Thermocouple_Channel()
{
    this->t = NULL;
}

Thermocouple* Thermocouple_Channel::get_thermocouple_type()
{
    return t;
}
void Thermocouple_Channel::set_thermocouple_type(Thermocouple* new_t)
{
    t = new_t;
}

void Thermocouple_Channel::setup_channel(Thermocouple* new_t, uint16_t thermocouple_channel, uint16_t rtd_channel, uint16_t calibration_channel)
{

    this->t = new_t;
    this->thermocouple_channel = thermocouple_channel;
    this->rtd_channel = rtd_channel;
    this->calibration_channel = calibration_channel;
    this->calibration_current = 0.0005;
}

CN0391::CN0391(PinName cs) : ad7124(cs)
{

}

void CN0391::set_thermocouple_type(channel_t ch, Thermocouple* new_t)
{
    tc[ch].setup_channel(new_t, ch * 2, (ch * 2) + 1, CAL_CHANNEL);
}


float CN0391::read_channel(channel_t ch)
{
    int32_t data;

    enable_current_source(tc[ch].rtd_channel);
    enable_channel(tc[ch].rtd_channel);
    start_single_conversion();

    if (ad7124.WaitForConvReady(10000) == -3) {
        pc.printf("TIMEOUT");
        return 0;
    }

    ad7124.ReadData(&data);
    disable_channel(tc[ch].rtd_channel);
    float volt = data_to_voltage(data >> 8);
    float R1 = (volt / tc[ch].calibration_current) - R2;
    float temp = RTD_CONVERSION(R1);
    float mv_cold_junction = tc[ch].t->convert_inv(temp);
    //disable_current_source(tc[ch].rtd_channel);
#ifdef DEBUG_MODE
    pc.printf("Resistance of RTD on channel %d is: %f\r\n", ch , R1);
    pc.printf("Temperature of RTD on channel %d is: %f\r\n", ch, temp);
    pc.printf("mV equivalent(poly method) of thermocouple is %f \r\n", mv_cold_junction);
#endif
    // read thermocouple
    enable_channel(tc[ch].thermocouple_channel);
    start_single_conversion();

    if (ad7124.WaitForConvReady(10000) == -3) {
        pc.printf("TIMEOUT");
        return 0;
    }

    ad7124.ReadData(&data);
    disable_channel(tc[ch].thermocouple_channel);


    volt = data_to_voltage(data >> 8);
    float mv = volt * 1000 + mv_cold_junction;
    temp = tc[ch].t->convert(mv);
#ifdef DEBUG_MODE
    pc.printf("mV of thermocouple on channel %d is: %f\r\n", ch, volt * 1000.0);
    pc.printf("mV(compensated) of thermocouple on channel %d is: %f\r\n", ch, mv);
    pc.printf("Temperature on channel %d is: %f\r\n", ch, temp);
#endif
    return temp;
}
float CN0391::calibrate(channel_t ch)
{
    int32_t data;
    enable_current_source(tc[ch].rtd_channel);
    enable_channel(tc[ch].calibration_channel); // calibration channel
    start_single_conversion();
    if (ad7124.WaitForConvReady(10000) == -3) {
        pc.printf("TIMEOUT");
        return 0;
    }

    ad7124.ReadData(&data);
    disable_channel(tc[ch].calibration_channel);
    //disable_current_source(tc[ch].rtd_channel);
    float volt = data_to_voltage(data >> 8);
    tc[ch].calibration_current = volt / R2;

#ifdef DEBUG_MODE
    pc.printf("Calibration current for channel %d is: %f \r\n", ch, tc[ch].calibration_current);
#endif
    return tc[ch].calibration_current;

}


float CN0391::data_to_voltage(uint32_t data)
{
    data = data & 0xFFFFFF;
    return ((data / static_cast<float>(0xFFFFFF / 2)) - 1) * (2.5 / 1);
}

void CN0391::enable_channel(int channel)
{
    AD7124::ad7124_registers regNr = static_cast<AD7124::ad7124_registers> (AD7124::AD7124_Channel_0 + channel); //Select ADC_Control register
    uint32_t setValue = ad7124.ReadDeviceRegister(regNr);
    setValue |= (uint32_t) AD7124_CH_MAP_REG_CH_ENABLE;  //Enable channel
    setValue &= 0xFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);    // Write data to ADC
    wait_ms(ms_delay);
}

void CN0391::disable_channel(int channel)
{
    AD7124::ad7124_registers regNr = static_cast<AD7124::ad7124_registers> (AD7124::AD7124_Channel_0 + channel); //Select ADC_Control register
    uint32_t setValue = ad7124.ReadDeviceRegister(regNr);
    setValue &= (~(uint32_t) AD7124_CH_MAP_REG_CH_ENABLE);  //Enable channel
    setValue &= 0xFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);    // Write data to ADC
    wait_ms(ms_delay);
}

void CN0391::enable_current_source(int current_source_channel)
{
    AD7124::ad7124_registers regNr = AD7124::AD7124_IOCon1; //Select ADC_Control register
    uint32_t setValue = ad7124.ReadDeviceRegister(regNr);
    setValue &= ~(AD7124_IO_CTRL1_REG_IOUT_CH0(0xF));
    setValue |= AD7124_IO_CTRL1_REG_IOUT_CH0(current_source_channel);// set IOUT0 current to 500uA
    setValue &= 0xFFFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);    // Write data to ADC
    wait_ms(ms_delay);
}

void CN0391::start_single_conversion()
{
    AD7124::ad7124_registers regNr = AD7124::AD7124_ADC_Control; //Select ADC_Control register
    uint32_t setValue = ad7124.ReadDeviceRegister(regNr);
    setValue &= 0xFFC3;
    setValue |= 0x04;               //single conversion;
    setValue |= 0x1600;
    setValue &= 0xFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);    // Write data to ADC*/
    wait_ms(ms_delay);
}

void CN0391::reset()
{
    ad7124.frequency(500000);
    ad7124.Reset();
    pc.printf("Reseted AD7124\r\n");
}

void CN0391::setup()
{
    ad7124.frequency(500000);
    ad7124.Setup();
}

void CN0391::init()
{
    uint32_t setValue;
    enum AD7124::ad7124_registers regNr;
    setup();
    wait_ms(ms_delay);

    /* Set Config_0 0x19*/
    regNr = AD7124::AD7124_Config_0;               //Select Config_0 register
    setValue = ad7124.ReadDeviceRegister(regNr);
    setValue |= AD7124_CFG_REG_BIPOLAR;     //Select bipolar operation
    setValue |= AD7124_CFG_REG_BURNOUT(0);  //Burnout current source off
    setValue |= AD7124_CFG_REG_REF_BUFP;
    setValue |= AD7124_CFG_REG_REF_BUFM;
    setValue |= AD7124_CFG_REG_AIN_BUFP;    //Buffer AIN5
    setValue |= AD7124_CFG_REG_AINN_BUFM;   //Buffer AIN4
    setValue |= AD7124_CFG_REG_REF_SEL(2); //Select REFIN1(+)/REFIN1(-)  internal reference
    setValue |= AD7124_CFG_REG_PGA(0);
    setValue &= 0xFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);   // Write data to ADC

    /* Set Channel_0 register 0x09*/
    regNr = AD7124::AD7124_Channel_0;
    setValue = ad7124.ReadDeviceRegister(regNr);
    //setValue |= (uint32_t) AD7124_CH_MAP_REG_CH_ENABLE;  //Enable channel0
    setValue &= (~(uint32_t)AD7124_CH_MAP_REG_CH_ENABLE);
    setValue |= AD7124_CH_MAP_REG_SETUP(0);             // Select setup0
    setValue |= AD7124_CH_MAP_REG_AINP(0);         // Set AIN4 as positive input
    setValue |= AD7124_CH_MAP_REG_AINM(15);         // Set AIN5 as negative input
    setValue &= 0xFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);   // Write data to ADC

    regNr = AD7124::AD7124_Channel_1;
    setValue = ad7124.ReadDeviceRegister(regNr);
    //setValue |= (uint32_t) AD7124_CH_MAP_REG_CH_ENABLE;  //Enable channel0
    setValue &= (~(uint32_t)AD7124_CH_MAP_REG_CH_ENABLE);
    setValue |= AD7124_CH_MAP_REG_SETUP(0);             // Select setup0
    setValue |= AD7124_CH_MAP_REG_AINP(1);         // Set AIN4 as positive input
    setValue |= AD7124_CH_MAP_REG_AINM(15);         // Set AIN5 as negative input
    setValue &= 0xFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);   // Write data to ADC

    regNr = AD7124::AD7124_Channel_2;
    setValue = ad7124.ReadDeviceRegister(regNr);
    //setValue |= (uint32_t) AD7124_CH_MAP_REG_CH_ENABLE;  //Enable channel0
    setValue &= (~(uint32_t)AD7124_CH_MAP_REG_CH_ENABLE);
    setValue |= AD7124_CH_MAP_REG_SETUP(0);             // Select setup0
    setValue |= AD7124_CH_MAP_REG_AINP(2);         // Set AIN4 as positive input
    setValue |= AD7124_CH_MAP_REG_AINM(15);         // Set AIN5 as negative input
    setValue &= 0xFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);   // Write data to ADC

    regNr = AD7124::AD7124_Channel_3;
    setValue = ad7124.ReadDeviceRegister(regNr);
    //setValue |= (uint32_t) AD7124_CH_MAP_REG_CH_ENABLE;  //Enable channel0
    setValue &= (~(uint32_t)AD7124_CH_MAP_REG_CH_ENABLE);
    setValue |= AD7124_CH_MAP_REG_SETUP(0);             // Select setup0
    setValue |= AD7124_CH_MAP_REG_AINP(3);         // Set AIN4 as positive input
    setValue |= AD7124_CH_MAP_REG_AINM(15);         // Set AIN5 as negative input
    setValue &= 0xFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);   // Write data to ADC

    regNr = AD7124::AD7124_Channel_4;
    setValue = ad7124.ReadDeviceRegister(regNr);
    //setValue |= (uint32_t) AD7124_CH_MAP_REG_CH_ENABLE;  //Enable channel0
    setValue &= (~(uint32_t)AD7124_CH_MAP_REG_CH_ENABLE);
    setValue |= AD7124_CH_MAP_REG_SETUP(0);             // Select setup0
    setValue |= AD7124_CH_MAP_REG_AINP(4);         // Set AIN4 as positive input
    setValue |= AD7124_CH_MAP_REG_AINM(15);         // Set AIN5 as negative input
    setValue &= 0xFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);   // Write data to ADC


    regNr = AD7124::AD7124_Channel_5;
    setValue = ad7124.ReadDeviceRegister(regNr);
    //setValue |= (uint32_t) AD7124_CH_MAP_REG_CH_ENABLE;  //Enable channel0
    setValue &= (~(uint32_t)AD7124_CH_MAP_REG_CH_ENABLE);
    setValue |= AD7124_CH_MAP_REG_SETUP(0);             // Select setup0
    setValue |= AD7124_CH_MAP_REG_AINP(5);         // Set AIN4 as positive input
    setValue |= AD7124_CH_MAP_REG_AINM(15);         // Set AIN5 as negative input
    setValue &= 0xFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);   // Write data to ADC


    regNr = AD7124::AD7124_Channel_6;
    setValue = ad7124.ReadDeviceRegister(regNr);
    //setValue |= (uint32_t) AD7124_CH_MAP_REG_CH_ENABLE;  //Enable channel0
    setValue &= (~(uint32_t)AD7124_CH_MAP_REG_CH_ENABLE);
    setValue |= AD7124_CH_MAP_REG_SETUP(0);             // Select setup0
    setValue |= AD7124_CH_MAP_REG_AINP(6);         // Set AIN4 as positive input
    setValue |= AD7124_CH_MAP_REG_AINM(15);         // Set AIN5 as negative input
    setValue &= 0xFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);   // Write data to ADC

    regNr = AD7124::AD7124_Channel_7;
    setValue = ad7124.ReadDeviceRegister(regNr);
    //setValue |= (uint32_t) AD7124_CH_MAP_REG_CH_ENABLE;  //Enable channel0
    setValue &= (~(uint32_t)AD7124_CH_MAP_REG_CH_ENABLE);
    setValue |= AD7124_CH_MAP_REG_SETUP(0);             // Select setup0
    setValue |= AD7124_CH_MAP_REG_AINP(7);         // Set AIN4 as positive input
    setValue |= AD7124_CH_MAP_REG_AINM(15);         // Set AIN5 as negative input
    setValue &= 0xFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);   // Write data to ADC

    regNr = AD7124::AD7124_Channel_8;
    setValue = ad7124.ReadDeviceRegister(regNr);
    //setValue |= (uint32_t) AD7124_CH_MAP_REG_CH_ENABLE;  //Enable channel0
    setValue &= (~(uint32_t)AD7124_CH_MAP_REG_CH_ENABLE);
    setValue |= AD7124_CH_MAP_REG_SETUP(0);             // Select setup0
    setValue |= AD7124_CH_MAP_REG_AINP(14);         // Set AIN4 as positive input
    setValue |= AD7124_CH_MAP_REG_AINM(15);         // Set AIN5 as negative input
    setValue &= 0xFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);   // Write data to ADC


    /* Set Config_0 0x19*/

#ifdef  CALIBRATION
    // start calibration
    regNr = AD7124::AD7124_Offset_0;
    setValue = 0x800000;
    ad7124.WriteDeviceRegister(regNr, setValue);// Write data to ADC

// internal fullscale before zero scale
    pc.printf("\r\n Gain before cali :%x", ad7124.ReadDeviceRegister(AD7124::AD7124_Gain_0));
    regNr = AD7124::AD7124_ADC_Control;//Select ADC_Control register
    setValue = AD7124_ADC_CTRL_REG_MODE(6);
    setValue |= AD7124_ADC_CTRL_REG_REF_EN;
    setValue &= 0xFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);// Write data to ADC
    //dut.WaitForConvReady(10000);
    wait_ms(2000);

    pc.printf("\r\n Gain:%x", ad7124.ReadDeviceRegister(AD7124::AD7124_Gain_0));

    pc.printf("\r\n Offset before cali:%x", ad7124.ReadDeviceRegister(AD7124::AD7124_Offset_0));
// internal zeroscale
    regNr = AD7124::AD7124_ADC_Control;//Select ADC_Control register
    setValue = AD7124_ADC_CTRL_REG_MODE(5);
    setValue |= AD7124_ADC_CTRL_REG_REF_EN;
    setValue &= 0xFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);// Write data to ADC
    wait_ms(2000);
    pc.printf("\r\n Offset:%x\r\n", ad7124.ReadDeviceRegister(AD7124::AD7124_Offset_0));

    // end of calibration

#endif

    /* Set IO_Control_1 0x03 */
    regNr = AD7124::AD7124_IOCon1;               //Select IO_Control_1 register
    setValue = ad7124.ReadDeviceRegister(regNr);
    setValue |= AD7124_IO_CTRL1_REG_IOUT0(0x4);// set IOUT0 current to 500uA
    setValue |= AD7124_IO_CTRL1_REG_IOUT_CH0(0x1);
    setValue &= 0xFFFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);// Write data to ADC

    /* Set ADC_Control 0x01 */
    regNr = AD7124::AD7124_ADC_Control;            //Select ADC_Control register
    setValue = ad7124.ReadDeviceRegister(regNr);
    setValue |= AD7124_ADC_CTRL_REG_DATA_STATUS; // set data status bit in order to check on which channel the conversion is
    setValue |= AD7124_ADC_CTRL_REG_REF_EN;
    setValue &= 0xFFC3;
    setValue |= AD7124_ADC_CTRL_REG_MODE(1);
    setValue &= 0xFFFF;
    ad7124.WriteDeviceRegister(regNr, setValue);    // Write data to ADC
    wait_ms(ms_delay);
}





