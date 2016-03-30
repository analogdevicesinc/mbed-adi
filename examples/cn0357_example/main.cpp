/**
*   @file     main.cpp
*   @brief    Toxic gas sensor shield application
*   @version  V0.1
*   @author   ADI
*   @date     March 2015
**/

#include "mbed.h"
#include "cn0357.h"

/** @mainpage
 * CN0357 single-supply, low noise, portable gas detector circuit using an
 * electrochemical sensor. The Alphasense CO-AX carbon monoxide sensor is used
 * in this example. Electrochemical sensors offer several advantages for
 * instruments that detect or measure the concentration of many toxic gases.
 * Most sensors are gas specific and have usable resolutions under one part per
 * million (ppm) of gas concentration.
 *
 * EVAL-CN0357 example program for MBED platforms. To achieve connection between
 * the shield and the ST Nucleo, the following connections need to be made:
 *
 * - ICSP pin 1 -> pin 12 of the Arduino header
 * - ICSP pin 3 -> pin 13 of the Arduino header
 * - ICSP pin 4 -> pin 11 of the Arduino header
 * - if VIN is not supplied, 5V to VIN
 *
 * @page CN0357 CN0357 Analog Devices wiki page
 * @brief link->
 * https://wiki.analog.com/resources/eval/user-guides/eval-adicup360/hardware/cn0357
 * @page AD7790 AD7790 datasheet
 * @brief link ->
 * http://www.analog.com/media/en/technical-documentation/data-sheets/AD7790.pdf
 * @page AD5270 AD5270 datasheet
 * @brief link ->
 * http://www.analog.com/media/en/technical-documentation/data-sheets/AD5270_5271.pdf
 *
 */
const float SENSOR_RANGE = 2000; ///< gas sensor range in PPM
const float SENSOR_SENSITIVITY = (65 * pow(10, -9)); ///< gas sensor sensitivity in A/ppm - 65 nA/ppm

Serial pc(USBTX, USBRX); ///< Serial interface to the pc

/**
 @brief Displays CN0357 circuit readings and data to the UART

 @param ui16Data - ADC data register value to be displayed
 @param fData1   - ADC input voltage reading to be displayed
 @param fdata2   - Gas Concentration reading to be displayed

 **/
void display_data(uint16_t ui16Data, float fData1, float fdata2)
{

    pc.printf("\r\nADC Data Register Value = %#08x", ui16Data); /** Send valid ADC data register value*/
    pc.printf("\r\nADC Input Voltage input = %f V", fData1); /** Send valid voltage input value */
    pc.printf("\r\nGas Concentration = %f PPM", fdata2); /** Send valid gas concentration value */

    pc.printf("\r\n");
}


/**
 * Project entry-point - initializes the CN0357 shield, reads the data when the ADC is ready and outputs the sensor
 * value in PPM
 */
int main()
{
    /* Main variables */


    CN0357 cn0357;
    cn0357.init(SENSOR_RANGE, SENSOR_SENSITIVITY);

    /* Infinite loop */
    while (1) {

        uint8_t ui8Status_Reg = cn0357.read_adc_status(); //  Read ADC Status Register

        if (ui8Status_Reg == 0x08) { //  Checks if ADC data is available
            uint16_t ui16Adcdata = cn0357.read_sensor();
            float fAdcVoltage    = cn0357.data_to_voltage(ui16Adcdata); //  Convert ADC data to voltage
            float fConcentration = cn0357.calc_ppm(fAdcVoltage); //  Convert voltage to Gas concentration
            display_data(ui16Adcdata, fAdcVoltage, fConcentration); //  Display data thru UART

            // printf("OneshotRead: %f PPM \r\n", cn0357.readPPM());
        }

        wait_ms(1000);
    }

    /* Infinite loop, never returns. */
}

