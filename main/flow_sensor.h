#ifndef FLOW_SENSOR_H
#define FLOW_SENSOR_H

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define FLOW_EDGE_QUEUE_LEN 32

// two channels
typedef enum {
    FLOW_CHANNEL_1,
    FLOW_CHANNEL_2,
} flow_channel_t;

// timestamped edge
typedef struct {
    flow_channel_t channel;
    int64_t timestamp_us;
} flow_edge_event_t;

// initialize pcnt units
void flow_sensor_init();

// raw PCNT totalizer count for a channel
int flow_sensor_get_count(flow_channel_t channel);

// gets amount of times the pcnt unit has surpassed the limit
// extra since unit config has this enabled, but can verify with this
uint32_t flow_sensor_get_overflow_count(flow_channel_t channel);

// timestamp queue one entry per pulse edge
QueueHandle_t flow_sensor_get_edge_queue(void);

#endif