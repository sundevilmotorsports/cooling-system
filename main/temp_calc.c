#include <math.h>

#include "temp_calc.h"

#define KELVIN_OFFSET 273.15f

// mapping
static const rt_point_t RT[] = {
    { -20.0f, 28146.0f }, { -10.0f, 15873.0f }, {   0.0f, 9256.0f },
    {  10.0f,  5572.0f }, {  20.0f,  3457.0f }, {  25.0f, 2830.0f },
    {  30.0f,  2205.0f }, {  40.0f,  1443.0f }, {  50.0f,  992.0f },
    {  60.0f,   660.0f }, {  70.0f,   475.0f }, {  80.0f,  329.0f },
    {  90.0f,   244.0f }, { 100.0f,   175.0f }, { 110.0f,  134.0f },
    { 120.0f,    99.0f }, { 140.0f,    60.0f }, { 160.0f,   47.0f },
};

#define RT_COUNT ((int)(sizeof(RT) / sizeof(RT[0])))

// raw counts * resolution
float temp_calc_volts(int16_t counts) {
    return (float)counts * TEMP_CALC_LSB_VOLTS;
}


float temp_calc_ntc_ohms(float volts, float r_bias_ohms, float v_supply) {
    // shorted sensor
    if (!(volts > 0.0f)) {
        return NAN;
    }

    // open circuit
    if (!(volts < v_supply)) {
        return NAN;
    }

    return r_bias_ohms * volts / (v_supply - volts);
}

float temp_calc_ntc_c(float r_ohms) {
    // check range
    if (!(r_ohms <= RT[0].r_ohms) || !(r_ohms >= RT[RT_COUNT - 1].r_ohms)) {
        return NAN;
    }

    // find point 
    int i = 0;
    while (i < RT_COUNT - 2 && r_ohms < RT[i + 1].r_ohms) {
        i++;
    }

    // piecewise beta model
    float t = logf(r_ohms / RT[i].r_ohms) / logf(RT[i + 1].r_ohms / RT[i].r_ohms);
    float inv_t0 = 1.0f / (RT[i].t_c + KELVIN_OFFSET);
    float inv_t1 = 1.0f / (RT[i + 1].t_c + KELVIN_OFFSET);

    return 1.0f / (inv_t0 + t * (inv_t1 - inv_t0)) - KELVIN_OFFSET;
}
