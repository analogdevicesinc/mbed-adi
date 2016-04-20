#include "mbed.h"
#include <stdio.h>
#include <vector>
#include <string>
#include "CN0357_Diag.h"

extern Serial pc;
extern vector<string> cmdbuffer;

CN0357_Diag::CN0357_Diag(CN0357& cn) : dut(cn)
{

}
void CN0357_Diag::set_RDAC()
{
    float res = strtof(cmdbuffer[1].c_str(), NULL);
    dut.set_RDAC_value(res);
    pc.printf("Wrote %f", res);

}
void CN0357_Diag::read_ppm(void)
{
    pc.printf("Wrote %f", dut.read_ppm());
}

void CN0357_Diag::set_sensor_param(void)
{
    float range = strtof(cmdbuffer[1].c_str(), NULL);
    float sens = strtof(cmdbuffer[2].c_str(), NULL);
    sens = sens * pow(10, -9);
    pc.printf("Suggested RDAC val: %f ", dut.set_sensor_parameters(range, sens));
}
