#include <stdint.h>
#include "mbed.h"
#include "ADXL362.h"

ADXL362::ADXL362(PinName CS, PinName MOSI, PinName MISO, PinName SCK) :
    adxl362(MOSI, MISO, SCK), cs(CS), int1(NULL), int2(NULL), int1_poll(NC), int2_poll(
        NC)
{
    cs = true; // cs is active low
    adxl362.format(8, _SPI_MODE);
    temp_stored_in_fifo = false;
    int1_act_low = true;
    int2_act_low = true;
}

void ADXL362::frequency(int hz)
{
    adxl362.frequency(hz);
}

void ADXL362::reset()
{
    adxl362.format(8, _SPI_MODE);
    cs = false;
    // Writing Code 0x52 (representing the letter, R, in ASCII or unicode) to this register immediately resets the ADXL362.
    write_reg(SOFT_RESET, 0x52);
    cs = true;
    // A latency of approximately 0.5 ms is required after soft reset.
}

void ADXL362::write_reg(ADXL362_register_t reg, uint8_t data)
{
    adxl362.format(8, _SPI_MODE);
    cs = false;
    adxl362.write(WRITE_REG_CMD);
    adxl362.write(static_cast<uint8_t>(reg));
    adxl362.write(static_cast<uint8_t>(data));
    cs = true;
}

uint8_t ADXL362::read_reg(ADXL362_register_t reg)
{
    uint8_t ret_val;
    adxl362.format(8, _SPI_MODE);
    cs = false;
    adxl362.write(READ_REG_CMD);
    adxl362.write(static_cast<uint8_t>(reg));
    ret_val = adxl362.write(DUMMY_BYTE);
    cs = true;
    return ret_val;
}

void ADXL362::write_reg_u16(ADXL362_register_t reg, uint16_t data)
{
    adxl362.format(8, _SPI_MODE);

    cs = false;
    adxl362.write(WRITE_REG_CMD);
    adxl362.write(static_cast<uint8_t>(reg));
    adxl362.write(static_cast<uint8_t>(data & 0xff));
    adxl362.write(static_cast<uint8_t>((data & 0xff00) >> 8));

    cs = true;
}
uint16_t ADXL362::read_reg_u16(ADXL362_register_t reg)
{
    uint16_t ret_val = 0;
    adxl362.format(8, _SPI_MODE);

    cs = false;
    adxl362.write(READ_REG_CMD);
    adxl362.write(static_cast<uint8_t>(reg));
    ret_val = adxl362.write(DUMMY_BYTE);
    ret_val = ret_val | (adxl362.write(DUMMY_BYTE) << 8);
    cs = true;

    return ret_val;
}

uint64_t ADXL362::scan()
{
    uint64_t ret_val = 0;
    uint16_t x, y, z, t = 0;

    adxl362.format(8, _SPI_MODE);
    cs = false;
    adxl362.write(READ_REG_CMD);
    adxl362.write(static_cast<uint8_t>(XDATA_L));

    x = adxl362.write(DUMMY_BYTE);
    x = x | static_cast<uint16_t>(adxl362.write(DUMMY_BYTE) << 8);
    y = adxl362.write(DUMMY_BYTE);
    y = y | static_cast<uint16_t>(adxl362.write(DUMMY_BYTE) << 8);
    z = adxl362.write(DUMMY_BYTE);
    z = z | static_cast<uint16_t>(adxl362.write(DUMMY_BYTE) << 8);
    t = adxl362.write(DUMMY_BYTE);
    t = t | static_cast<uint16_t>(adxl362.write(DUMMY_BYTE) << 8);
    // format xxyyzztt
    ret_val = static_cast<uint64_t>(x) << 48;
    ret_val |= static_cast<uint64_t>(y) << 32;
    ret_val |= static_cast<uint64_t>(z) << 16;
    ret_val |= static_cast<uint64_t>(t);
    cs = true;
    return ret_val;
}

uint8_t ADXL362::scanx_u8()
{
    return read_reg(XDATA);
}
uint16_t ADXL362::scanx()
{
    return read_reg_u16(XDATA_L);
}
uint8_t ADXL362::scany_u8()
{
    return read_reg(YDATA);
}

uint16_t ADXL362::scany()
{
    return read_reg_u16(YDATA_L);
}
uint8_t ADXL362::scanz_u8()
{
    return read_reg(ZDATA);
}

uint16_t ADXL362::scanz()
{
    return read_reg_u16(ZDATA_L);
}
uint16_t ADXL362::scant()
{
    return read_reg_u16(TEMP_L);
}

void ADXL362::set_mode(ADXL362_modes_t mode)
{
    uint8_t reg_val;
    reg_val = read_reg(POWER_CTL);
    reg_val = reg_val | static_cast<uint8_t>(mode);
    write_reg(POWER_CTL, reg_val);
}

void ADXL362::set_activity_threshold(uint16_t threshold)
{
    write_reg_u16(THRESH_ACT_L, threshold);
}
void ADXL362::set_activity_time(uint8_t time)
{
    write_reg(TIME_ACT, time);
}
void ADXL362::set_inactivity_threshold(uint16_t threshold)
{
    write_reg_u16(THRESH_INACT_L, threshold);
}
void ADXL362::set_inactivity_time(uint16_t time)
{
    write_reg_u16(TIME_INACT_L, time);
}

void ADXL362::set_polling_interrupt1_pin(PinName in, uint8_t data,
        PinMode pull)
{
    if ((data & 0x7F) != 0) {
        write_reg(INTMAP1, data);
        int1_poll = DigitalIn(in);
        int1_poll.mode(pull);
        if (data & 0x80) {
            int1_act_low = true;
        } else {
            int1_act_low = false;
        }
    }
}

void ADXL362::set_polling_interrupt2_pin(PinName in, uint8_t data,
        PinMode pull)
{
    if ((data & 0x7F) != 0) {
        write_reg(INTMAP2, data);
        int2_poll = DigitalIn(in);
        int2_poll.mode(pull);
        if (data & 0x80) {
            int2_act_low = true;
        } else {
            int2_act_low = false;
        }
    }
}

bool ADXL362::get_int1()
{
    return (int1_poll.read() != int1_act_low);  // boolean XOR
}

bool ADXL362::get_int2()
{
    return (int1_poll.read() != int1_act_low); // boolean XOR
}

void ADXL362::set_interrupt1_pin(PinName in, uint8_t data,
                                 void (*callback_rising)(void), void (*callback_falling)(void), PinMode pull)
{
    if ((data & 0x7F) != 0) {
        write_reg(INTMAP1, data);
        //int1 = InterruptIn(in);
        delete int1;
        int1 = new InterruptIn(in);
        int1->mode(pull);
        if(callback_falling != NULL) int1->fall(callback_falling);
        if(callback_rising != NULL) int1->rise(callback_rising);
        if (data & 0x80) {
            int1_act_low = true;
        } else {
            int1_act_low = false;
        }
    }
}
void ADXL362::set_interrupt2_pin(PinName in, uint8_t data,
                                 void (*callback_rising)(void), void (*callback_falling)(void), PinMode pull)
{
    if ((data & 0x7F) != 0) {
        write_reg(INTMAP2, data);
        delete int2;
        int2 = new InterruptIn(in);
        int2->mode(pull);
        if(callback_falling != NULL) int2->fall(callback_falling);
        if(callback_rising != NULL) int2->rise(callback_rising);
        if (data & 0x80) {
            int2_act_low = true;
        } else {
            int2_act_low = false;
        }
    }
}

void ADXL362::enable_interrupt1()
{
    int1->enable_irq();
}

void ADXL362::enable_interrupt2()
{
    int2->enable_irq();
}

void ADXL362::disable_interrupt1()
{
    int1->disable_irq();
}

void ADXL362::disable_interrupt2()
{
    int2->disable_irq();
}

void ADXL362::set_act_inact_ctl_reg(uint8_t data)
{
    write_reg(ACT_INACT_CTL, data);
}

void ADXL362::set_power_ctl_reg(uint8_t data)
{
    write_reg(POWER_CTL, data);
}

void ADXL362::set_filter_ctl_reg(uint8_t data)
{
    write_reg(FILTER_CTL, data);
}

uint8_t ADXL362::read_status()
{
    return read_reg(STATUS);
}

uint16_t ADXL362::fifo_read_nr_of_entries()
{
    return read_reg_u16(FIFO_ENTRIES_L);
}

void ADXL362::fifo_setup(bool store_temp, ADXL362_FIFO_modes_t mode, uint16_t nr_of_entries)
{
    uint8_t fifo_ctl = static_cast<uint8_t>(mode);
    temp_stored_in_fifo = store_temp;

    fifo_ctl = fifo_ctl | (static_cast<uint8_t>(temp_stored_in_fifo) << 2);

    if (nr_of_entries > 0xff) {
        fifo_ctl = fifo_ctl | static_cast<uint8_t>(AH);
    }
    write_reg(FIFO_CONTROL, fifo_ctl);
    write_reg(FIFO_SAMPLES, static_cast<uint8_t>(nr_of_entries & 0xff));

}

uint16_t ADXL362::fifo_read_u16()
{
    uint16_t ret_val = 0;
    adxl362.format(8, _SPI_MODE);

    cs = false;
    adxl362.write(READ_FIFO_CMD);
    ret_val = adxl362.write(DUMMY_BYTE);
    ret_val = (ret_val) | static_cast<uint16_t>(adxl362.write(DUMMY_BYTE) << 8);
    cs = true;

    return ret_val;
}

uint64_t ADXL362::fifo_scan()
{

    uint64_t ret_val = 0;
    uint16_t x = 0, y = 0, z = 0, dummy, t = 0, sample_type;

    adxl362.format(8, _SPI_MODE);
    cs = false;
    adxl362.write(READ_FIFO_CMD);
    uint8_t samples = (temp_stored_in_fifo) ? 4 : 3;
    for(uint8_t i = 0; i < samples; i++) {
        dummy = adxl362.write(DUMMY_BYTE);
        dummy = dummy | static_cast<uint16_t>(adxl362.write(DUMMY_BYTE) << 8);
        sample_type = (dummy & 0xc000) >> 14;
        dummy = dummy & 0x3fff;
        switch(sample_type) {
            case 0: // x
                x = dummy;
                break;
            case 1: // y
                y = dummy;
                break;
            case 2: // z
                z = dummy;
                break;
            case 3: // temp
                t = dummy;
                break;
        }

    }

    // format xxyyzztt
    ret_val = static_cast<uint64_t> (x) << 48;
    ret_val |= static_cast<uint64_t>(y) << 32;
    ret_val |= static_cast<uint64_t>(z) << 16;
    ret_val |= static_cast<uint64_t>(t);
    cs = true;
    return ret_val;
}
