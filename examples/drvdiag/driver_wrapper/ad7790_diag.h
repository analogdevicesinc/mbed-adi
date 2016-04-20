/*
 * ad7790.h
 *
 *  Created on: Apr 15, 2016
 *      Author: asuciu
 */

#ifndef AD7790_DIAG_H_
#define AD7790_DIAG_H_

#include "AD7790.h"

class AD7790_Diag
{

public:
    AD7790_Diag(AD7790& ad);
    void init(void);
    void reset(void);
    void write_mode(void);
    void write_filter(void);
    void read_mode(void);
    void read_filter(void);
    void read_data(void);
    void read_status(void);
    void read_u16(void);
    void read_voltage(void);
    void set_continous_mode(void);
    void set_reference_voltage(void);
    void set_channel(void);

private:
    AD7790& dut;
};

/*

*/



#endif /* AD7790_DIAG_H_ */
