#ifndef TEMP_CALC_H
#define TEMP_CALC_H

#include <stdint.h>

#define TEMP_CALC_LSB_VOLTS   (4.096f / 32768.0f) 
#define TEMP_CALC_R_BIAS_OHMS 510.0f              
#define TEMP_CALC_V_SUPPLY    3.3f   

typedef struct {
    float t_c;
    float r_ohms;
} rt_point_t;

float temp_calc_volts(int16_t counts);

float temp_calc_ntc_ohms(float volts, float r_bias_ohms, float v_supply);

float temp_calc_ntc_c(float r_ohms);

#endif
