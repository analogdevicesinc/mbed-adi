#include "mbed.h"
#ifndef _THERMOCOUPLE_H_
#define _THERMOCOUPLE_H_

#define DEFINE_LOOKUP_TABLES
#ifdef DEFINE_LOOKUP_TABLES
#define TYPE_B_LUT
#define TYPE_E_LUT
#define TYPE_J_LUT
#define TYPE_K_LUT
#define TYPE_N_LUT
#define TYPE_R_LUT
#define TYPE_S_LUT
#define TYPE_T_LUT
#endif

class Thermocouple
{
private:

public:
    typedef struct {
        float min_voltage_range;
        float max_voltage_range;
        float coef[16];
        float power[16];
        int n;
    } thermocouple_poly_subrange;
    Thermocouple();
    virtual ~Thermocouple();
    static float convert(float voltage, const thermocouple_poly_subrange range[], const int n);
    static float lookup(const int32_t *lut, float voltage, uint16_t size, int16_t offset);
    virtual float convert(float voltage) = 0;
    virtual float convert_inv(float temp) = 0;
    virtual float lookup(float voltage) = 0;
    virtual float lookup_inv(float temp) = 0;

};




class Thermocouple_Type_B : public Thermocouple
{
public:
    ~Thermocouple_Type_B();
    static const thermocouple_poly_subrange inv_poly[2];
    static const int inv_poly_size;
    float convert_inv(float temp);

    static const thermocouple_poly_subrange poly[2];
    static const int poly_size;
    float convert(float voltage);
#ifdef TYPE_B_LUT
    static const int32_t lut[];
    static const int16_t lut_offset;
    static const uint16_t lut_size;
    float lookup(float voltage);
    float lookup_inv(float temp);
#endif
};


class Thermocouple_Type_E : public Thermocouple
{
public:
    ~Thermocouple_Type_E();
    static const thermocouple_poly_subrange inv_poly[2];
    static const int inv_poly_size;
    float convert_inv(float temp);

    static const thermocouple_poly_subrange poly[2];
    static const int poly_size;
    float convert(float voltage);
#ifdef TYPE_E_LUT
    static const int32_t lut[];
    static const int16_t lut_offset;
    static const uint16_t lut_size;
    float lookup(float voltage);
    float lookup_inv(float temp);
#endif
};


class Thermocouple_Type_J : public Thermocouple
{
public:
    ~Thermocouple_Type_J();
    static const thermocouple_poly_subrange inv_poly[2];
    static const int inv_poly_size;
    float convert_inv(float temp);

    static const thermocouple_poly_subrange poly[3];
    static const int poly_size;
    float convert(float voltage);
#ifdef TYPE_J_LUT
    static const int32_t lut[];
    static const int16_t lut_offset;
    static const uint16_t lut_size;
    float lookup(float voltage);
    float lookup_inv(float temp);
#endif
};


class Thermocouple_Type_K : public Thermocouple
{
public:
    ~Thermocouple_Type_K();
    static const thermocouple_poly_subrange inv_poly[2];
    static const int inv_poly_size;
    float convert_inv(float temp);

    static const thermocouple_poly_subrange poly[3];
    static const int poly_size;
    float convert(float voltage);
#ifdef TYPE_K_LUT
    static const int32_t lut[];
    static const int16_t lut_offset;
    static const uint16_t lut_size;
    float lookup(float voltage);
    float lookup_inv(float temp);
#endif
};


class Thermocouple_Type_N : public Thermocouple
{
public:
    ~Thermocouple_Type_N();
    static const thermocouple_poly_subrange inv_poly[2];
    static const int inv_poly_size;
    float convert_inv(float temp);

    static const thermocouple_poly_subrange poly[3];
    static const int poly_size;
    float convert(float voltage);
#ifdef TYPE_N_LUT
    static const int32_t lut[];
    static const int16_t lut_offset;
    static const uint16_t lut_size;
    float lookup(float voltage);
    float lookup_inv(float temp);
#endif
};


class Thermocouple_Type_R : public Thermocouple
{
public:
    ~Thermocouple_Type_R();
    static const thermocouple_poly_subrange inv_poly[3];
    static const int inv_poly_size;
    float convert_inv(float temp);

    static const thermocouple_poly_subrange poly[4];
    static const int poly_size;
    float convert(float voltage);
#ifdef TYPE_R_LUT
    static const int32_t lut[];
    static const int16_t lut_offset;
    static const uint16_t lut_size;
    float lookup(float voltage);
    float lookup_inv(float temp);
#endif
};


class Thermocouple_Type_S : public Thermocouple
{
public:
    ~Thermocouple_Type_S();
    static const thermocouple_poly_subrange inv_poly[3];
    static const int inv_poly_size;
    float convert_inv(float temp);

    static const thermocouple_poly_subrange poly[4];
    static const int poly_size;
    float convert(float voltage);
#ifdef TYPE_S_LUT
    static const int32_t lut[];
    static const int16_t lut_offset;
    static const uint16_t lut_size;
    float lookup(float voltage);
    float lookup_inv(float temp);
#endif
};


class Thermocouple_Type_T : public Thermocouple
{
public:
    ~Thermocouple_Type_T();
    static const thermocouple_poly_subrange inv_poly[2];
    static const int inv_poly_size;
    float convert_inv(float temp);

    static const thermocouple_poly_subrange poly[2];
    static const int poly_size;
    float convert(float voltage);
#ifdef TYPE_T_LUT
    static const int32_t lut[];
    static const int16_t lut_offset;
    static const uint16_t lut_size;
    float lookup(float voltage);
    float lookup_inv(float temp);
#endif
};



#endif
