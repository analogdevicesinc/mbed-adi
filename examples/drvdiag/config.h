
#define AD7790_PRESENT
#define AD5270_PRESENT
#define CN0357_PRESENT
#define SPI_LOW_LEVEL

#ifdef AD7790_PRESENT
#include "AD7790.h"
#include "ad7790_diag.h"
#endif

#ifdef AD5270_PRESENT
#include "ad5270.h"
#include "ad5270_diag.h"
#endif

#ifdef CN0357_PRESENT
#include "cn0357.h"
#include "CN0357_Diag.h"
#endif

using namespace std;
//------------------------------------
// Hyperterminal configuration
// 9600 bauds, 8-bit data, no parity
//------------------------------------

#ifdef SPI_LOW_LEVEL
DigitalOut CSA_pin(D8); // cs adc
DigitalOut CSR_pin(D6); // cs rdac
SPI spibus(SPI_MOSI, SPI_MISO, SPI_SCK);
#endif

#ifdef AD7790_PRESENT
AD7790 ad7790(1.2, D8);
AD7790_Diag ad7790diag(ad7790);
#endif

#ifdef AD5270_PRESENT
AD5270 ad5270(D6, 20000);
AD5270_Diag ad5270diag(ad5270);
#endif

#ifdef  CN0357_PRESENT
CN0357 cn0357;
CN0357_Diag cn0357diag(cn0357);
#endif