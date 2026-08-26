#ifndef CAN_TX_H
#define CAN_TX_H

#include "flow_sensor.h"

// latest computed values per channel, written by flow_processor
typedef struct {
    float rate_lpm[2];
    float total_volume_l[2];
} flow_data_t;

void can_init();

// called by flow_processor after each update
void update_flow(flow_channel_t channel, float rate_lpm, float total_volume_l);

// called by can_tx task to get a coherent copy before encoding
void get_flow(flow_data_t *out);

#endif
