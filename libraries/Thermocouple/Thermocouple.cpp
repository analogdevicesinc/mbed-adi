#include "Thermocouple.h"

// http://srdata.nist.gov/its90/download/allcoeff.tab

Thermocouple::Thermocouple(){}
Thermocouple::~Thermocouple() {}

float Thermocouple::lookup(const int32_t *lut, float voltage, uint16_t size, int16_t offset) {
    uint16_t first = 0;
    uint16_t last = size - 1;
    uint16_t middle = (first + last) / 2;
    int32_t integer_voltage = int32_t(voltage*1000);
    while (first <= last) {
        if (lut[middle] < integer_voltage)
            first = middle + 1;
        else if (lut[middle] == integer_voltage) {
            return static_cast<float>(middle + offset);
        } else
            last = middle - 1;

        middle = (first + last) / 2;
    }
    if (first > last)
        return static_cast<float>(first+offset);

    return 0; // should never get here
}

float Thermocouple::convert(float voltage, const thermocouple_poly_subrange range[], const int n) {
    int range_id = 0;
    float temperature=0;
    for(range_id = 0 ; range_id<n;range_id++)
    {
        if(voltage > range[range_id].min_voltage_range && voltage <= range[range_id].max_voltage_range)
            break;
    }

    for (int i = 0; i < range[range_id].n; i++) {
        temperature += (range[range_id].coef[i] * pow(10,range[range_id].power[i])) * pow(voltage, i);
    }
    return temperature;
}




const int Thermocouple_Type_B::inv_poly_size = 2;
const Thermocouple::thermocouple_poly_subrange Thermocouple_Type_B::inv_poly[2] =
{
	{     0.000,   630.615,// characteristic curve for temp range between 0.000, and 630.615,
	{0.000000000000,-0.246508183460,0.590404211710,-0.132579316360,0.156682919010,-0.169445292400,0.629903470940,},
	{          0,         -3,         -5,         -8,        -11,        -14,        -18,},
	7 },
	{   630.615,  1820.000,// characteristic curve for temp range between 630.615, and 1820.000,
	{-0.389381686210,0.285717474700,-0.848851047850,0.157852801640,-0.168353448640,0.111097940130,-0.445154310330,0.989756408210,-0.937913302890,},
	{          1,         -1,         -4,         -6,         -9,        -12,        -16,        -20,        -24,},
	9 }
};

float Thermocouple_Type_B::convert_inv(float temp)
{
	return Thermocouple::convert(temp, inv_poly, inv_poly_size);
}

float Thermocouple_Type_B::lookup_inv(float temp)
{
#ifdef TYPE_B_LUT
	if((temp+lut_offset)>lut_size)
		return lut[lut_size-1];
	else
		return lut[(uint16_t)temp+lut_offset];
#else
	/* NOT IMPLEMENTED */
	return 0;
#endif
}
const int Thermocouple_Type_B::poly_size = 2;
const Thermocouple::thermocouple_poly_subrange Thermocouple_Type_B::poly[2] =
{
	{      0.291,       2.431, // characteristic curve for mV range between 0.291 and 2.431
	{  9.8423321,  6.9971500, -8.4765304,  1.0052644, -8.3345952,  4.5508542, -1.5523037,  2.9886750, -2.4742860,},
	{          1,          2,          2,          3,          2,          2,          2,          1,          0,},
	9 },
	{      2.431,      13.820, // characteristic curve for mV range between 2.431 and 13.820
	{  2.1315071,  2.8510504, -5.2742887,  9.9160804, -1.2965303,  1.1195870, -6.0625199,  1.8661696, -2.4878585,},
	{          2,          2,          1,          0,          0,         -1,         -3,         -4,         -6,},
	9 }
};

Thermocouple_Type_B::~Thermocouple_Type_B()
{

}

float Thermocouple_Type_B::convert(float voltage)
{
	return Thermocouple::convert(voltage, poly, poly_size);
}

float Thermocouple_Type_B::lookup(float voltage)
{
#ifdef TYPE_B_LUT
	return Thermocouple::lookup(lut, voltage, lut_size, lut_offset);
#else
	/* NOT IMPLEMENTED */
	return 0;
#endif
}
const int Thermocouple_Type_E::inv_poly_size = 2;
const Thermocouple::thermocouple_poly_subrange Thermocouple_Type_E::inv_poly[2] =
{
	{  -270.000,     0.000,// characteristic curve for temp range between -270.000, and 0.000,
	{0.000000000000,0.586655087080,0.454109771240,-0.779980486860,-0.258001608430,-0.594525830570,-0.932140586670,-0.102876055340,-0.803701236210,-0.439794973910,-0.164147763550,-0.396736195160,-0.558273287210,-0.346578420130,},
	{          0,         -1,         -4,         -6,         -7,         -9,        -11,        -12,        -15,        -17,        -19,        -22,        -25,        -28,},
	14 },
	{     0.000,  1000.000,// characteristic curve for temp range between 0.000, and 1000.000,
	{0.000000000000,0.586655087100,0.450322755820,0.289084072120,-0.330568966520,0.650244032700,-0.191974955040,-0.125366004970,0.214892175690,-0.143880417820,0.359608994810,},
	{          0,         -1,         -4,         -7,         -9,        -12,        -15,        -17,        -20,        -23,        -27,},
	11 }
};

float Thermocouple_Type_E::convert_inv(float temp)
{
	return Thermocouple::convert(temp, inv_poly, inv_poly_size);
}

float Thermocouple_Type_E::lookup_inv(float temp)
{
#ifdef TYPE_E_LUT
	if((temp+lut_offset)>lut_size)
		return lut[lut_size-1];
	else
		return lut[(uint16_t)temp+lut_offset];
#else
	/* NOT IMPLEMENTED */
	return 0;
#endif
}
const int Thermocouple_Type_E::poly_size = 2;
const Thermocouple::thermocouple_poly_subrange Thermocouple_Type_E::poly[2] =
{
	{     -8.825,       0.000, // characteristic curve for mV range between -8.825 and 0.000
	{  0.0000000,  1.6977288, -4.3514970, -1.5859697, -9.2502871, -2.6084314, -4.1360199, -3.4034030, -1.1564890,  0.0000000,},
	{          0,          1,         -1,         -1,         -2,         -2,         -3,         -4,         -5,          0,},
	10 },
	{      0.000,      76.373, // characteristic curve for mV range between 0.000 and 76.373
	{  0.0000000,  1.7057035, -2.3301759,  6.5435585, -7.3562749, -1.7896001,  8.4036165, -1.3735879,  1.0629823, -3.2447087,},
	{          0,          1,         -1,         -3,         -5,         -6,         -8,         -9,        -11,        -14,},
	10 }
};

Thermocouple_Type_E::~Thermocouple_Type_E()
{

}

float Thermocouple_Type_E::convert(float voltage)
{
	return Thermocouple::convert(voltage, poly, poly_size);
}

float Thermocouple_Type_E::lookup(float voltage)
{
#ifdef TYPE_E_LUT
	return Thermocouple::lookup(lut, voltage, lut_size, lut_offset);
#else
	/* NOT IMPLEMENTED */
	return 0;
#endif
}
const int Thermocouple_Type_J::inv_poly_size = 2;
const Thermocouple::thermocouple_poly_subrange Thermocouple_Type_J::inv_poly[2] =
{
	{  -210.000,   760.000,// characteristic curve for temp range between -210.000, and 760.000,
	{0.000000000000,0.503811878150,0.304758369300,-0.856810657200,0.132281952950,-0.170529583370,0.209480906970,-0.125383953360,0.156317256970,},
	{          0,         -1,         -4,         -7,         -9,        -12,        -15,        -18,        -22,},
	9 },
	{   760.000,  1200.000,// characteristic curve for temp range between 760.000, and 1200.000,
	{0.296456256810,-0.149761277860,0.317871039240,-0.318476867010,0.157208190040,-0.306913690560,},
	{          3,          1,         -2,         -5,         -8,        -12,},
	6 }
};

float Thermocouple_Type_J::convert_inv(float temp)
{
	return Thermocouple::convert(temp, inv_poly, inv_poly_size);
}

float Thermocouple_Type_J::lookup_inv(float temp)
{
#ifdef TYPE_J_LUT
	if((temp+lut_offset)>lut_size)
		return lut[lut_size-1];
	else
		return lut[(uint16_t)temp+lut_offset];
#else
	/* NOT IMPLEMENTED */
	return 0;
#endif
}
const int Thermocouple_Type_J::poly_size = 3;
const Thermocouple::thermocouple_poly_subrange Thermocouple_Type_J::poly[3] =
{
	{     -8.095,       0.000, // characteristic curve for mV range between -8.095 and 0.000
	{  0.0000000,  1.9528268, -1.2286185, -1.0752178, -5.9086933, -1.7256713, -2.8131513, -2.3963370, -8.3823321,},
	{          0,          1,          0,          0,         -1,         -1,         -2,         -3,         -5,},
	9 },
	{      0.000,      42.919, // characteristic curve for mV range between 0.000 and 42.919
	{   0.000000,   1.978425,  -2.001204,   1.036969,  -2.549687,   3.585153,  -5.344285,   5.099890,   0.000000,},
	{          0,          1,         -1,         -2,         -4,         -6,         -8,        -10,          0,},
	9 },
	{     42.919,      69.553, // characteristic curve for mV range between 42.919 and 69.553
	{-3.11358187, 3.00543684,-9.94773230, 1.70276630,-1.43033468, 4.73886084, 0.00000000, 0.00000000, 0.00000000,},
	{          3,          2,          0,         -1,         -3,         -6,          0,          0,          0,},
	9 }
};

Thermocouple_Type_J::~Thermocouple_Type_J()
{

}

float Thermocouple_Type_J::convert(float voltage)
{
	return Thermocouple::convert(voltage, poly, poly_size);
}

float Thermocouple_Type_J::lookup(float voltage)
{
#ifdef TYPE_J_LUT
	return Thermocouple::lookup(lut, voltage, lut_size, lut_offset);
#else
	/* NOT IMPLEMENTED */
	return 0;
#endif
}
const int Thermocouple_Type_K::inv_poly_size = 2;
const Thermocouple::thermocouple_poly_subrange Thermocouple_Type_K::inv_poly[2] =
{
	{  -270.000,     0.000,// characteristic curve for temp range between -270.000, and 0.000,
	{0.000000000000,0.394501280250,0.236223735980,-0.328589067840,-0.499048287770,-0.675090591730,-0.574103274280,-0.310888728940,-0.104516093650,-0.198892668780,-0.163226974860,},
	{          0,         -1,         -4,         -6,         -8,        -10,        -12,        -14,        -16,        -19,        -22,},
	11 },
	{     0.000,  1372.000,// characteristic curve for temp range between 0.000, and 1372.000,
	{-0.176004136860,0.389212049750,0.185587700320,-0.994575928740,0.318409457190,-0.560728448890,0.560750590590,-0.320207200030,0.971511471520,-0.121047212750,},
	{         -1,         -1,         -4,         -7,         -9,        -12,        -15,        -18,        -22,        -25,},
	10 }
};

float Thermocouple_Type_K::convert_inv(float temp)
{
	return Thermocouple::convert(temp, inv_poly, inv_poly_size);
}

float Thermocouple_Type_K::lookup_inv(float temp)
{
#ifdef TYPE_K_LUT
	if((temp+lut_offset)>lut_size)
		return lut[lut_size-1];
	else
		return lut[(uint16_t)temp+lut_offset];
#else
	/* NOT IMPLEMENTED */
	return 0;
#endif
}
const int Thermocouple_Type_K::poly_size = 3;
const Thermocouple::thermocouple_poly_subrange Thermocouple_Type_K::poly[3] =
{
	{     -5.891,       0.000, // characteristic curve for mV range between -5.891 and 0.000
	{  0.0000000,  2.5173462, -1.1662878, -1.0833638, -8.9773540, -3.7342377, -8.6632643, -1.0450598, -5.1920577,  0.0000000,},
	{          0,          1,          0,          0,         -1,         -1,         -2,         -2,         -4,          0,},
	10 },
	{      0.000,      20.644, // characteristic curve for mV range between 0.000 and 20.644
	{   0.000000,   2.508355,   7.860106,  -2.503131,   8.315270,  -1.228034,   9.804036,  -4.413030,   1.057734,  -1.052755,},
	{          0,          1,         -2,         -1,         -2,         -2,         -4,         -5,         -6,         -8,},
	10 },
	{     20.644,      54.886, // characteristic curve for mV range between 20.644 and 54.886
	{  -1.318058,   4.830222,  -1.646031,   5.464731,  -9.650715,   8.802193,  -3.110810,   0.000000,   0.000000,   0.000000,},
	{          2,          1,          0,         -2,         -4,         -6,         -8,          0,          0,          0,},
	10 }
};

Thermocouple_Type_K::~Thermocouple_Type_K()
{

}

float Thermocouple_Type_K::convert(float voltage)
{
	return Thermocouple::convert(voltage, poly, poly_size);
}

float Thermocouple_Type_K::lookup(float voltage)
{
#ifdef TYPE_K_LUT
	return Thermocouple::lookup(lut, voltage, lut_size, lut_offset);
#else
	/* NOT IMPLEMENTED */
	return 0;
#endif
}
const int Thermocouple_Type_N::inv_poly_size = 2;
const Thermocouple::thermocouple_poly_subrange Thermocouple_Type_N::inv_poly[2] =
{
	{  -270.000,     0.000,// characteristic curve for temp range between -270.000, and 0.000,
	{0.000000000000,0.261591059620,0.109574842280,-0.938411115540,-0.464120397590,-0.263033577160,-0.226534380030,-0.760893007910,-0.934196678350,},
	{          0,         -1,         -4,         -7,        -10,        -11,        -13,        -16,        -19,},
	9 },
	{     0.000,  1300.000,// characteristic curve for temp range between 0.000, and 1300.000,
	{0.000000000000,0.259293946010,0.157101418800,0.438256272370,-0.252611697940,0.643118193390,-0.100634715190,0.997453389920,-0.608632456070,0.208492293390,-0.306821961510,},
	{          0,         -1,         -4,         -7,         -9,        -12,        -14,        -18,        -21,        -24,        -28,},
	11 }
};

float Thermocouple_Type_N::convert_inv(float temp)
{
	return Thermocouple::convert(temp, inv_poly, inv_poly_size);
}

float Thermocouple_Type_N::lookup_inv(float temp)
{
#ifdef TYPE_N_LUT
	if((temp+lut_offset)>lut_size)
		return lut[lut_size-1];
	else
		return lut[(uint16_t)temp+lut_offset];
#else
	/* NOT IMPLEMENTED */
	return 0;
#endif
}
const int Thermocouple_Type_N::poly_size = 3;
const Thermocouple::thermocouple_poly_subrange Thermocouple_Type_N::poly[3] =
{
	{     -3.990,       0.000, // characteristic curve for mV range between -3.990 and 0.000
	{  0.0000000,  3.8436847,  1.1010485,  5.2229312,  7.2060525,  5.8488586,  2.7754916,  7.7075166,  1.1582665,  7.3138868,},
	{          0,          1,          0,          0,          0,          0,          0,         -1,         -1,         -3,},
	10 },
	{      0.000,      20.613, // characteristic curve for mV range between 0.000 and 20.613
	{    0.00000,    3.86896,   -1.08267,    4.70205,   -2.12169,   -1.17272,    5.39280,   -7.98156,    0.00000,    0.00000,},
	{          0,          1,          0,         -2,         -6,         -4,         -6,         -8,          0,          0,},
	10 },
	{     20.613,      47.513, // characteristic curve for mV range between 20.613 and 47.513
	{   1.972485,   3.300943,  -3.915159,   9.855391,  -1.274371,   7.767022,   0.000000,   0.000000,   0.000000,   0.000000,},
	{          1,          1,         -1,         -3,         -4,         -7,          0,          0,          0,          0,},
	10 }
};

Thermocouple_Type_N::~Thermocouple_Type_N()
{

}

float Thermocouple_Type_N::convert(float voltage)
{
	return Thermocouple::convert(voltage, poly, poly_size);
}

float Thermocouple_Type_N::lookup(float voltage)
{
#ifdef TYPE_N_LUT
	return Thermocouple::lookup(lut, voltage, lut_size, lut_offset);
#else
	/* NOT IMPLEMENTED */
	return 0;
#endif
}
const int Thermocouple_Type_R::inv_poly_size = 3;
const Thermocouple::thermocouple_poly_subrange Thermocouple_Type_R::inv_poly[3] =
{
	{   -50.000,  1064.180,// characteristic curve for temp range between -50.000, and 1064.180,
	{0.000000000000,0.528961729765,0.139166589782,-0.238855693017,0.356916001063,-0.462347666298,0.500777441034,-0.373105886191,0.157716482367,-0.281038625251,},
	{          0,         -2,         -4,         -7,        -10,        -13,        -16,        -19,        -22,        -26,},
	10 },
	{  1064.180,  1664.500,// characteristic curve for temp range between 1064.180, and 1664.500,
	{0.295157925316,-0.252061251332,0.159564501865,-0.764085947576,0.205305291024,-0.293359668173,},
	{          1,         -2,         -4,         -8,        -11,        -15,},
	6 },
	{  1664.500,  1768.100,// characteristic curve for temp range between 1664.500, and 1768.100,
	{0.152232118209,-0.268819888545,0.171280280471,-0.345895706453,-0.934633971046,},
	{          3,          0,         -3,         -7,        -14,},
	5 }
};

float Thermocouple_Type_R::convert_inv(float temp)
{
	return Thermocouple::convert(temp, inv_poly, inv_poly_size);
}

float Thermocouple_Type_R::lookup_inv(float temp)
{
#ifdef TYPE_R_LUT
	if((temp+lut_offset)>lut_size)
		return lut[lut_size-1];
	else
		return lut[(uint16_t)temp+lut_offset];
#else
	/* NOT IMPLEMENTED */
	return 0;
#endif
}
const int Thermocouple_Type_R::poly_size = 4;
const Thermocouple::thermocouple_poly_subrange Thermocouple_Type_R::poly[4] =
{
	{     -0.226,       1.923, // characteristic curve for mV range between -0.226 and 1.923
	{  0.0000000,  1.8891380, -9.3835290,  1.3068619, -2.2703580,  3.5145659, -3.8953900,  2.8239471, -1.2607281,  3.1353611, -3.3187769,},
	{          0,          2,          1,          2,          2,          2,          2,          2,          2,          1,          0,},
	11 },
	{      1.923,      13.228, // characteristic curve for mV range between 1.923 and 13.228
	{1.334584505,1.472644573,-1.844024844,4.031129726,-6.249428360,6.468412046,-4.458750426,1.994710149,-5.313401790,6.481976217,0.000000000,},
	{          1,          2,          1,          0,         -1,         -2,         -3,         -4,         -6,         -8,          0,},
	11 },
	{     11.361,      19.739, // characteristic curve for mV range between 11.361 and 19.739
	{-8.199599416,1.553962042,-8.342197663,4.279433549,-1.191577910,1.492290091,0.000000000,0.000000000,0.000000000,0.000000000,0.000000000,},
	{          1,          2,          0,         -1,         -2,         -4,          0,          0,          0,          0,          0,},
	11 },
	{     19.739,      21.103, // characteristic curve for mV range between 19.739 and 21.103
	{3.406177836,-7.023729171,5.582903813,-1.952394635,2.560740231,0.000000000,0.000000000,0.000000000,0.000000000,0.000000000,0.000000000,},
	{          4,          3,          2,          1,         -1,          0,          0,          0,          0,          0,          0,},
	11 }
};

Thermocouple_Type_R::~Thermocouple_Type_R()
{

}

float Thermocouple_Type_R::convert(float voltage)
{
	return Thermocouple::convert(voltage, poly, poly_size);
}

float Thermocouple_Type_R::lookup(float voltage)
{
#ifdef TYPE_R_LUT
	return Thermocouple::lookup(lut, voltage, lut_size, lut_offset);
#else
	/* NOT IMPLEMENTED */
	return 0;
#endif
}
const int Thermocouple_Type_S::inv_poly_size = 3;
const Thermocouple::thermocouple_poly_subrange Thermocouple_Type_S::inv_poly[3] =
{
	{   -50.000,  1064.180,// characteristic curve for temp range between -50.000, and 1064.180,
	{0.000000000000,0.540313308631,0.125934289740,-0.232477968689,0.322028823036,-0.331465196389,0.255744251786,-0.125068871393,0.271443176145,},
	{          0,         -2,         -4,         -7,        -10,        -13,        -16,        -19,        -23,},
	9 },
	{  1064.180,  1664.500,// characteristic curve for temp range between 1064.180, and 1664.500,
	{0.132900444085,0.334509311344,0.654805192818,-0.164856259209,0.129989605174,},
	{          1,         -2,         -5,         -8,        -13,},
	5 },
	{  1664.500,  1768.100,// characteristic curve for temp range between 1664.500, and 1768.100,
	{0.146628232636,-0.258430516752,0.163693574641,-0.330439046987,-0.943223690612,},
	{          3,          0,         -3,         -7,        -14,},
	5 }
};

float Thermocouple_Type_S::convert_inv(float temp)
{
	return Thermocouple::convert(temp, inv_poly, inv_poly_size);
}

float Thermocouple_Type_S::lookup_inv(float temp)
{
#ifdef TYPE_S_LUT
	if((temp+lut_offset)>lut_size)
		return lut[lut_size-1];
	else
		return lut[(uint16_t)temp+lut_offset];
#else
	/* NOT IMPLEMENTED */
	return 0;
#endif
}
const int Thermocouple_Type_S::poly_size = 4;
const Thermocouple::thermocouple_poly_subrange Thermocouple_Type_S::poly[4] =
{
	{     -0.235,       1.874, // characteristic curve for mV range between -0.235 and 1.874
	{ 0.00000000, 1.84949460,-8.00504062, 1.02237430,-1.52248592, 1.88821343,-1.59085941, 8.23027880,-2.34181944, 2.79786260,},
	{          0,          2,          1,          2,          2,          2,          2,          1,          1,          0,},
	10 },
	{      1.874,      11.950, // characteristic curve for mV range between 1.874 and 11.950
	{1.291507177,1.466298863,-1.534713402,3.145945973,-4.163257839,3.187963771,-1.291637500,2.183475087,-1.447379511,8.211272125,},
	{          1,          2,          1,          0,         -1,         -2,         -3,         -5,         -7,         -9,},
	10 },
	{     10.332,      17.536, // characteristic curve for mV range between 10.332 and 17.536
	{-8.087801117,1.621573104,-8.536869453,4.719686976,-1.441693666,2.081618890,0.000000000,0.000000000,0.000000000,0.000000000,},
	{          1,          2,          0,         -1,         -2,         -4,          0,          0,          0,          0,},
	10 },
	{     17.536,      18.693, // characteristic curve for mV range between 17.536 and 18.693
	{5.333875126,-1.235892298,1.092657613,-4.265693686,6.247205420,0.000000000,0.000000000,0.000000000,0.000000000,0.000000000,},
	{          4,          4,          3,          1,         -1,          0,          0,          0,          0,          0,},
	10 }
};

Thermocouple_Type_S::~Thermocouple_Type_S()
{

}

float Thermocouple_Type_S::convert(float voltage)
{
	return Thermocouple::convert(voltage, poly, poly_size);
}

float Thermocouple_Type_S::lookup(float voltage)
{
#ifdef TYPE_S_LUT
	return Thermocouple::lookup(lut, voltage, lut_size, lut_offset);
#else
	/* NOT IMPLEMENTED */
	return 0;
#endif
}
const int Thermocouple_Type_T::inv_poly_size = 2;
const Thermocouple::thermocouple_poly_subrange Thermocouple_Type_T::inv_poly[2] =
{
	{  -270.000,     0.000,// characteristic curve for temp range between -270.000, and 0.000,
	{0.000000000000,0.387481063640,0.441944343470,0.118443231050,0.200329735540,0.901380195590,0.226511565930,0.360711542050,0.384939398830,0.282135219250,0.142515947790,0.487686622860,0.107955392700,0.139450270620,0.797951539270,},
	{          0,         -1,         -4,         -6,         -7,         -9,        -10,        -12,        -14,        -16,        -18,        -21,        -23,        -26,        -30,},
	15 },
	{     0.000,   400.000,// characteristic curve for temp range between 0.000, and 400.000,
	{0.000000000000,0.387481063640,0.332922278800,0.206182434040,-0.218822568460,0.109968809280,-0.308157587720,0.454791352900,-0.275129016730,},
	{          0,         -1,         -4,         -6,         -8,        -10,        -13,        -16,        -19,},
	9 }
};

float Thermocouple_Type_T::convert_inv(float temp)
{
	return Thermocouple::convert(temp, inv_poly, inv_poly_size);
}

float Thermocouple_Type_T::lookup_inv(float temp)
{
#ifdef TYPE_T_LUT
	if((temp+lut_offset)>lut_size)
		return lut[lut_size-1];
	else
		return lut[(uint16_t)temp+lut_offset];
#else
	/* NOT IMPLEMENTED */
	return 0;
#endif
}
const int Thermocouple_Type_T::poly_size = 2;
const Thermocouple::thermocouple_poly_subrange Thermocouple_Type_T::poly[2] =
{
	{     -5.603,       0.000, // characteristic curve for mV range between -5.603 and 0.000
	{  0.0000000,  2.5949192, -2.1316967,  7.9018692,  4.2527777,  1.3304473,  2.0241446,  1.2668171,},
	{          0,          1,         -1,         -1,         -1,         -1,         -2,         -3,},
	8 },
	{      0.000,      20.872, // characteristic curve for mV range between 0.000 and 20.872
	{   0.000000,   2.592800,  -7.602961,   4.637791,  -2.165394,   6.048144,  -7.293422,   0.000000,},
	{          0,          1,         -1,         -2,         -3,         -5,         -7,          0,},
	8 }
};

Thermocouple_Type_T::~Thermocouple_Type_T()
{

}

float Thermocouple_Type_T::convert(float voltage)
{
	return Thermocouple::convert(voltage, poly, poly_size);
}

float Thermocouple_Type_T::lookup(float voltage)
{
#ifdef TYPE_T_LUT
	return Thermocouple::lookup(lut, voltage, lut_size, lut_offset);
#else
	/* NOT IMPLEMENTED */
	return 0;
#endif
}
