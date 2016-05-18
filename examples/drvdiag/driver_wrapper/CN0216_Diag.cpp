#include "mbed.h"
#include <stdio.h>
#include <vector>
#include <string>

#include "CN0216_Diag.h"

extern Serial pc;
extern vector<string> cmdbuffer;

CN0216_Diag::CN0216_Diag(CN0216& cn) : dut(cn)
{

}

void CN0216_Diag::init(void)
{
    uint16_t weight = strtol(cmdbuffer[1].c_str(), NULL, 10);
    pc.printf("CN0216 initialized with %d calibration weight", weight);
    dut.init(weight);
}

void CN0216_Diag::calibrate(void)
{
    uint8_t step = strtol(cmdbuffer[1].c_str(), NULL, 16);
    pc.printf("Calibrating step %d ..", step);
    dut.calibrate(static_cast<CN0216::CalibrationStep_t>(step));
    pc.printf(".. DONE", step);
}

void CN0216_Diag::read_weight(void)
{
    pc.printf("Read weight is %f", dut.read_weight());
}

