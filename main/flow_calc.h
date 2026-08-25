#ifndef FLOW_CALC_H
#define FLOW_CALC_H

#include <stdint.h>

typedef struct {
    int64_t last_edge_us; // determines valid edge
    float rate_lpm;       // last computed rate
} flow_calc_state_t;

// volume = pulses / liter * pulse_count
float volume(int32_t pulse_count, float k);

// rate
float rate();

void flow_calc_state_init(flow_calc_state_t *state);

// process rising edge
void flow_calc_process_edge(flow_calc_state_t *state, int64_t timestamp_us, float k);


#endif