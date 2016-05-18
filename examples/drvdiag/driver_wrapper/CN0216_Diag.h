
#ifndef CN0216_DIAG_H_
#define CN0216_DIAG_H_
#include "CN0216.h"

class CN0216_Diag
{
public:
    CN0216_Diag(CN0216& cn);

    void init(void);
    void calibrate(void);
    void read_weight(void);


private:
    CN0216& dut;
};


#endif /* CN0216_DIAG_H_ */
