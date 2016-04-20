#include "mbed.h"
#include <stdio.h>
#include <vector>
#include <string>
#include "AD7790_Diag.h"

extern Serial pc;
extern vector<string> cmdbuffer;

AD7790_Diag::AD7790_Diag(AD7790& ad) : dut(ad)
{

}

void AD7790_Diag::init()
{

}
void AD7790_Diag::reset()
{
    dut.reset();
    pc.printf("Reseted AD7790");
}

void AD7790_Diag::write_mode()
{
    uint8_t regVal = strtol(cmdbuffer[1].c_str(), NULL, 16);
    dut.write_mode_reg(regVal);
    pc.printf("Wrote mode");
}
void AD7790_Diag::read_mode()
{
    pc.printf("Mode reg: %x ", dut.read_mode_reg());
}

void AD7790_Diag::write_filter()
{
    uint8_t regVal = strtol(cmdbuffer[1].c_str(), NULL, 16);
    dut.write_filter_reg(regVal);
    pc.printf("Wrote filter");
}
void AD7790_Diag::read_filter()
{
    pc.printf("Returned: %x ", dut.read_filter_reg());
}
void AD7790_Diag::read_data()
{
    pc.printf("Data reg: %x ", dut.read_data_reg());
}

void AD7790_Diag::read_status()
{
    pc.printf("Status reg: %x ", dut.read_status_reg());
}

void AD7790_Diag::read_u16()
{
    pc.printf("Data reg: %x ", dut.read_u16());
}
void AD7790_Diag::read_voltage()
{
    pc.printf("Voltage: %f ", dut.read_voltage());
}
void AD7790_Diag::set_continous_mode()
{
    uint8_t regVal = strtol(cmdbuffer[1].c_str(), NULL, 16);
    dut.set_conversion_mode(static_cast<AD7790::AD7790Mode_t>(regVal));
    pc.printf("Mode set to %d", regVal);
}
void AD7790_Diag::set_reference_voltage()
{
    float ref = strtof(cmdbuffer[1].c_str(), NULL);
    dut.set_reference_voltage(ref);
    pc.printf("Reference Voltage set to %f", ref);
}

void AD7790_Diag::set_channel()
{
    uint8_t regVal = strtol(cmdbuffer[1].c_str(), NULL, 16);
    dut.set_channel(static_cast<AD7790::AD7790Channel_t>(regVal));
    pc.printf("Mode set to %d", regVal);
}
