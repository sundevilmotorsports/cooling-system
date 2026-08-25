#include <stdint.h>

#include "flow_sensor.h"

float volume(int32_t pulse_count, float k) {
    return k * pulse_count;
}

void flow_calc_state_init(flow_calc_state_t *state) {
    *state = (flow_calc_state_t){0};
}

void flow_calc_process_edge(flow_calc_state_t *state, int64_t timestamp_us, float k) {
    // just recovered from timeout
    if (*state->last_edge_us == -1) {
        *state->last_edge_us = timestamp_us;
        return;
    }

    float period = timestemp_us - state->last_edge_us;
    float rate = 6e7 / period / k; // convert microseconds per pulse to pulses per minute, pulses per min to liters per min
    *state->last_edge_us = timestamp_us;

    return rate;
}