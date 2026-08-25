#ifndef FLOW_CALC_H
#define FLOW_CALC_H

#include <stdint.h>

typedef struct {
    int64_t last_edge_us; // -1 = no valid edge yet (cold start / just timed out)
    float rate_lpm;       // last computed rate
} flow_calc_state_t;

void flow_calc_state_init(flow_calc_state_t *state);

// process one pulse edge
void flow_calc_process_edge(flow_calc_state_t *state, int64_t timestamp_us, float k_factor, int64_t min_period_us);

// call periodically to check for timrouts
void flow_calc_check_timeout(flow_calc_state_t *state, int64_t now_us, int64_t timeout_us);

float flow_calc_get_rate(const flow_calc_state_t *state);

// volume = pulse_count / k_factor
float flow_calc_get_volume(int32_t pulse_count, float k_factor);

#endif
