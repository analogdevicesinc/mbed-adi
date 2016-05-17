#ifndef ADXL362_H_
#define ADXL362_H_

#include "mbed.h"

class ADXL362
{
public:

    /* Temperature parameters */
    typedef enum {
        DEVID_AD = 0x00,
        DEVID_MST = 0x01,
        PARTID = 0x02,
        REVID = 0x03,
        XDATA = 0x08,
        YDATA = 0x09,
        ZDATA = 0x0A,
        STATUS = 0x0B,
        FIFO_ENTRIES_L = 0x0C,
        FIFO_ENTRIES_H = 0x0D,
        XDATA_L = 0x0E,
        XDATA_H = 0x0F,
        YDATA_L = 0x10,
        YDATA_H = 0x11,
        ZDATA_L = 0x12,
        ZDATA_H = 0x13,
        TEMP_L = 0x14,
        TEMP_H = 0x15,
        // Reserved = 0x16;
        // Reserved = 0x17;
        SOFT_RESET = 0x1F,
        THRESH_ACT_L = 0x20,
        THRESH_ACT_H = 0x21,
        TIME_ACT = 0x22,
        THRESH_INACT_L = 0x23,
        THRESH_INACT_H = 0x24,
        TIME_INACT_L = 0x25,
        TIME_INACT_H = 0x26,
        ACT_INACT_CTL = 0x27,
        FIFO_CONTROL = 0x28,
        FIFO_SAMPLES = 0x29,
        INTMAP1 = 0x2A,
        INTMAP2 = 0x2B,
        FILTER_CTL = 0x2C,
        POWER_CTL = 0x2D,
        SELF_TEST = 0x2E,
    } ADXL362_register_t;

    typedef enum {
        STANDBY = 0x00,
        MEASUREMENT = 0x02
    } ADXL362_modes_t;

    typedef enum {
        ERR_USER_REGS = 0x80,
        AWAKE = 0x40,
        INACT = 0x20,
        ACT = 0x10,
        FIFO_OVERRUN = 0x08,
        FIFO_WATERMARK = 0x04,
        FIFO_READY = 0x02,
        DATA_READY = 0x01
    } ADXL362_STATUS_reg_bits_t;

    typedef enum {
        LINKLOOP1 = 0x20,
        LINKLOOP0 = 0x10,
        DEFAULTMODE = 0x00,
        LINKED_MODE = 0x10,
        LOOP_MODE = 0x30,
        INACT_REF = 0x08,
        INACT_EN = 0x04,
        ACT_REF = 0x02,
        ACT_EN = 0x01
    } ADXL362_ACT_INACT_CTL_reg_bits_t;

    typedef enum {
        AH = 0x08,
        FIFO_TEMP = 0x04,
        FIFO_MODE1 = 0x02,
        FIFO_MODE = 0x01,
    } ADXL362_FIFO_CONTROL_reg_bits_t;

    typedef enum {
        FIFO_DISABLED = 0x00,
        FIFO_OLDEST = 0x01,
        FIFO_STREAM = 0x02,
        FIFO_TRIGGERED = 0x03,
    } ADXL362_FIFO_modes_t;

    typedef enum {
        INT_LOW = 0x80,
        INT_AWAKE = 0x40,
        INT_INACT = 0x20,
        INT_ACT = 0x10,
        INT_FIFO_OVERRUN = 0x08,
        INT_FIFO_WATERMARK = 0x04,
        INT_FIFO_READY = 0x02,
        INT_DATA_READY = 0x01
    } ADXL362_INTMAP_reg_bits_t;

    typedef enum {
        RANGE1 = 0x80,
        RANGE0 = 0x40,
        RANGE2G = 0x00,
        RANGE4G = 0x40,
        RANGE8G = 0x80,
        HALF_BW = 0x10,
        EXT_SAMPLE = 0x08,
        ODR2 = 0x04,
        ODR1 = 0x02,
        ODR0 = 0x01,
        ODR12HZ = 0x00,
        ODR25HZ = 0x01,
        ODR50Hz = 0x02,
        ODR100HZ = 0x03,
        ODR200Hz = 0x04,
        ODR400HZ = 0x07
    } ADXL362_FILTER_CTL_reg_bits_t;

    typedef enum {
        EXT_CLK = 0x40,
        LOW_NOISE1 = 0x20,
        LOW_NOISE0 = 0x10,
        NORMAL_OPERATION = 0x00,
        LOW_NOISE = 0x10,
        ULTRALOW_NOISE = 0x20,
        WAKEUP = 0x08,
        AUTOSLEEP = 0x04,
        MEASURE1 = 0x02,
        MEASURE0 = 0x01,
    } ADXL362_POWER_CTL_reg_bits_t;

    /** SPI configuration & constructor */
    ADXL362(PinName CS = SPI_CS, PinName MOSI = SPI_MOSI, PinName MISO =
                SPI_MISO, PinName SCK = SPI_SCK);
    void frequency(int hz);

    /** Low level SPI bus comm methods */
    void reset(void);
    void write_reg(ADXL362_register_t reg, uint8_t data);
    uint8_t read_reg(ADXL362_register_t reg);
    uint64_t scan();

    void set_power_ctl_reg(uint8_t data);
    void set_filter_ctl_reg(uint8_t data);

    uint8_t scanx_u8();
    uint16_t scanx();
    uint8_t scany_u8();
    uint16_t scany();
    uint8_t scanz_u8();
    uint16_t scanz();
    uint16_t scant();

    uint8_t read_status();
    void set_mode(ADXL362_modes_t mode);

    void set_activity_threshold(uint16_t threshold);
    void set_activity_time(uint8_t time);
    void set_inactivity_threshold(uint16_t threshold);
    void set_inactivity_time(uint16_t time);
    void set_act_inact_ctl_reg(uint8_t data);

    void set_interrupt1_pin(PinName in, uint8_t data, void (*callback_rising)(void), void (*callback_falling)(void), PinMode pull = PullNone);
    void set_interrupt2_pin(PinName in, uint8_t data, void (*callback_rising)(void), void (*callback_falling)(void), PinMode pull = PullNone);
    void enable_interrupt1();
    void enable_interrupt2();
    void disable_interrupt1();
    void disable_interrupt2();

    void set_polling_interrupt1_pin(PinName in, uint8_t data, PinMode pull = PullNone);
    void set_polling_interrupt2_pin(PinName in, uint8_t data, PinMode pull = PullNone);

    bool get_int1();
    bool get_int2();


    uint16_t fifo_read_nr_of_entries();
    void fifo_setup(bool store_temp, ADXL362_FIFO_modes_t mode, uint16_t nr_of_entries);
    uint16_t fifo_read_u16();
    uint64_t fifo_scan();





private:

    SPI adxl362;    ///< SPI instance of the AD7791
    DigitalOut cs; ///< DigitalOut instance for the chipselect of the AD7791
    uint16_t read_reg_u16(ADXL362_register_t reg);
    InterruptIn *int1;
    InterruptIn *int2;
    DigitalIn int1_poll;
    DigitalIn int2_poll;
    bool int1_act_low;
    bool int2_act_low;
    bool temp_stored_in_fifo;

    void write_reg_u16(ADXL362_register_t reg, uint16_t data);

    const static uint8_t DUMMY_BYTE = 0xAA;
    const static uint8_t WRITE_REG_CMD = 0x0A; // write register
    const static uint8_t READ_REG_CMD = 0x0B; // read register
    const static uint8_t READ_FIFO_CMD = 0x0D; // read FIFO
    const static uint8_t _SPI_MODE = 0;
};

#endif
