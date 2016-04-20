
#ifndef AD5270_DIAG_H_
#define AD5270_DIAG_H_
#include "ad5270.h"

class AD5270_Diag
{
public:
    AD5270_Diag(AD5270& ad);
    void enable_50TP_programming(void);
    void store_50TP(void);
    void disable_50TP_programming(void);

    void write_RDAC(void);
    void read_RDAC(void);
    void write_cmd(void);
    void set_HiZ(void);

    void read_50TP_last_address(void);
    void read_50TP_memory(void);

    void write_ctrl_reg(void);
    void read_ctrl_reg(void);

    void reset_RDAC(void);
    void change_mode(void);

    void write_wiper_reg(void);
    void read_wiper_reg(void);

private:
    AD5270& dut;
};


#endif /* AD5270_DIAG_H_ */
