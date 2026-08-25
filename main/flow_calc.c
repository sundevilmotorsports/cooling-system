#include "flow_calc.h"

void flow_calc_state_init(flow_calc_state_t *state) {
    state->last_edge_us = -1;
    state->rate_lpm = 0.0f;
}

void flow_calc_process_edge(flow_calc_state_t *state, int64_t timestamp_us, float k_factor, int64_t min_period_us) {
    // cold start / just recovered from timeout - no period to measure yet
    if (state->last_edge_us == -1) {
        state->last_edge_us = timestamp_us;
        return;
    }

    int64_t period_us = timestamp_us - state->last_edge_us;

    // noise - too short of a period
    if (period_us < min_period_us) {
        return;
    }

    // period (us/pulse), pulses/min, L/min
    state->rate_lpm = 6e7f / (float)period_us / k_factor;
    state->last_edge_us = timestamp_us;
}

void flow_calc_check_timeout(flow_calc_state_t *state, int64_t now_us, int64_t timeout_us) {
    if (state->last_edge_us == -1) {
        return; // already cold
    }
    if (now_us - state->last_edge_us > timeout_us) {
        state->last_edge_us = -1;
        state->rate_lpm = 0.0f;
    }
}

float flow_calc_get_rate(const flow_calc_state_t *state) {
    return state->rate_lpm;
}

float flow_calc_get_volume(int32_t pulse_count, float k_factor) {
    return (float)pulse_count / k_factor;
}
