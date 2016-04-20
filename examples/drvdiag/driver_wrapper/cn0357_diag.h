
#ifndef CN0357_DIAG_H_
#define CN0357_DIAG_H_
#include "CN0357.h"

class CN0357_Diag
{
public:
    CN0357_Diag(CN0357& cn);

    void set_RDAC(void);
    void read_ppm(void);
    void set_sensor_param(void);


private:
    CN0357& dut;
};


#endif /* AD5270_DIAG_H_ */
