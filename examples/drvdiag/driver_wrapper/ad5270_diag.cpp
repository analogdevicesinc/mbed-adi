#include "mbed.h"
#include <stdio.h>
#include <vector>
#include <string>
#include "AD5270_Diag.h"

extern Serial pc;
extern vector<string> cmdbuffer;

AD5270_Diag::AD5270_Diag(AD5270& ad) : dut(ad)
{

}
void AD5270_Diag::enable_50TP_programming()
{
    dut.enable_50TP_programming();
    pc.printf("Enabled 50TP prog");

}
void AD5270_Diag::store_50TP()
{
    dut.store_50TP();
    pc.printf("50TP stored");
}
void AD5270_Diag::disable_50TP_programming()
{
    dut.disable_50TP_programming();
    pc.printf("Disabled 50TP prog");
}

void AD5270_Diag::write_RDAC()
{
    float res = strtof(cmdbuffer[1].c_str(), NULL);
    dut.write_RDAC(res);
    pc.printf("Wrote %f", res);
}
void AD5270_Diag::read_RDAC()
{
    pc.printf("Read %f", dut.read_RDAC());
}
void AD5270_Diag::write_cmd()
{
    uint8_t reg = strtol(cmdbuffer[1].c_str(), NULL, 16);
    uint8_t regVal = strtol(cmdbuffer[2].c_str(), NULL, 16);
    pc.printf("Returned %x: ", dut.write_cmd(reg, regVal));
}

void AD5270_Diag::set_HiZ()
{
    pc.printf("SDO set to HiZ");
    dut.set_SDO_HiZ();
}

void AD5270_Diag::read_50TP_last_address(void)
{
    pc.printf("Returned %x:", dut.read_50TP_last_address());
}
void AD5270_Diag::read_50TP_memory(void)
{
    uint8_t reg = strtol(cmdbuffer[1].c_str(), NULL, 16);
    pc.printf("Returned %x", dut.read_50TP_memory(reg));
}

void AD5270_Diag::write_ctrl_reg(void)
{
    uint8_t val = strtol(cmdbuffer[1].c_str(), NULL, 16);
    dut.write_ctrl_reg(val);
    pc.printf("Wrote %x to ctrl_reg", val );
}
void AD5270_Diag::read_ctrl_reg(void)
{
    pc.printf("Read %x from ctrl_reg", dut.read_ctrl_reg());
}

void AD5270_Diag::reset_RDAC(void)
{
    dut.reset_RDAC();
    pc.printf("Resetted rdac");
}
void AD5270_Diag::change_mode(void)
{
    uint8_t val = strtol(cmdbuffer[1].c_str(), NULL, 16);
    dut.change_mode(static_cast<AD5270::AD5270Modes_t>(val));
    pc.printf("Changed mode to %x", val);
}

void AD5270_Diag::write_wiper_reg(void)
{
    uint16_t val = strtol(cmdbuffer[1].c_str(), NULL, 16);
    dut.write_wiper_reg(val);
    pc.printf("Wrote %x to wiper", val);
}
void AD5270_Diag::read_wiper_reg(void)
{
    pc.printf("Read %x from wiper", dut.read_wiper_reg());
}
