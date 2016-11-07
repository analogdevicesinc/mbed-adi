#include "CN0397_Diag.h"

CN0397_Diag::CN0397_Diag(CN0397& ad) :
    dut(ad)
{

}

void CN0397_Diag::init()
{
    dut.init();
}

void CN0397_Diag::reset()
{
    dut.ad7798.reset();
}

void CN0397_Diag::read()
{
    dut.set_app_data();
    dut.display_data();
}

