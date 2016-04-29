
#include "mbed.h"
#include "AD7791.h"

#ifndef CN0216_H_
#define CN0216_H_

class CN0216
{
public:
    typedef enum {
        ZERO_SCALE_CALIBRATION, ///< Calibration of the zero scale value
        FULL_SCALE_CALIBRATION, ///< Calibration of the full scale value
        COMPUTE_UNITS_PER_BIT   ///< Units per LSB computation
    } CalibrationStep_t;

    CN0216(PinName CSAD7791 = D8, PinName MOSI = SPI_MOSI, PinName MISO = SPI_MISO, PinName SCK = SPI_SCK);
    void init(float cal_weight = _DEFAULT_CAL_WEIGHT, uint8_t mode_val = _DEFAULT_MODE_VAL, uint8_t filter_val = _DEFAULT_FILTER_VAL);
    void calibrate(CalibrationStep_t cal);
    float compute_weight(uint32_t data);
    uint32_t read_u32();
    float read_weight();

private:

    const static int _NUMBER_OF_SAMPLES = 20; ///< Number of samples used in calibration
    const static int _DEFAULT_MODE_VAL = AD7791::MD1 | AD7791::MD0; // POWERDOWN MODE
    const static int _DEFAULT_FILTER_VAL = AD7791::FS0 | AD7791::FS1 | AD7791::FS2;
    const static int _DEFAULT_CAL_WEIGHT = 1000.0;

    AD7791 ad7791;
    float _cal_weight;
    uint32_t _zero_scale_value;
    uint32_t _full_scale_value;
    float _weight_units_per_bit;

};

#endif
