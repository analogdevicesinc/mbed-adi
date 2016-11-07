#ifndef _CN0391_H_
#define _CN0391_H_
#include "AD7124.h"
#include "Thermocouple.h"


/**
 * @brief Thermocouple_Channel class
 */
class Thermocouple_Channel
{
private:
public:
	/**
	 * @brief Thermocouple channel constructor
	 */
    Thermocouple_Channel();
    /**
     * @brief Constructs thermocouple channel using thermocouple type t
     */
    Thermocouple_Channel(Thermocouple *t);

    Thermocouple *t;
    uint16_t thermocouple_channel;
    uint16_t rtd_channel;
    uint16_t calibration_channel;
    float calibration_current;

	/**
	 * @brief gets thermocouple type
	 * @return thermocouple type
	 */
    Thermocouple* get_thermocouple_type();

	/**
	 * @brief sets new thermocouple type
	 * @param new_t new thermocouple type
	 */
    void set_thermocouple_type(Thermocouple* new_t);

    /**
     * @brief sets up thermocouple channel
     * @param new_t thermocouple type
     * @param thermocouple_channel - thermocouple ADC channel
     * @param rtd_channel - RTD ADC channel
     * @param calibration_channel - ADC channel used in calibration
     *
     */
    void setup_channel(Thermocouple* new_t, uint16_t thermocouple_channel, uint16_t rtd_channel, uint16_t calibration_channel);
};

class CN0391
{
private:
public:
    CN0391(PinName cs);

    typedef enum {
        CHANNEL_P1 = 0,
        CHANNEL_P2,
        CHANNEL_P3,
        CHANNEL_P4
    } channel_t;

    Thermocouple_Channel tc[4];

    /**
     * @brief sets channel with thermocouple type
     * @param ch - channel
     * @param new_t - thermocouple type
     */
    void set_thermocouple_type(channel_t ch, Thermocouple* new_t);

    /**
     * @brief Reads thermocouple channel
     * @param ch - channel
     */
    float read_channel(channel_t ch);

    /**
     * @brief Calibrate channel
     * @param ch - channel
     */
    float calibrate(channel_t ch);


    /**
     * @brief converts ADC  counts to voltage
     * @param data - ADC counts
     * @return voltage
     */
    float data_to_voltage(uint32_t data);

    /**
     * @brief enables thermocouple channel
     * @param channel
     */
    void enable_channel(int channel);

    /**
     * @brief disables thermocouple channel
     * @param channel
     */
    void disable_channel(int channel);

    /**
     * @brief enables ADC current source
     * @param current_source_channel
     */
    void enable_current_source(int current_source_channel);

    /**
     * @brief starts ADC single conversion
     */
    void start_single_conversion();

    /**
     * @brief resets the ADC
     */
    void reset();

    /**
     * @brief Performs ADC setup
     */
    void setup();

    /**
     * @brief Initializes the CN0391 shield
     */
    void init();

    AD7124 ad7124;


};
#endif
