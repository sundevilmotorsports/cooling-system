#ifndef FLOW_SENSOR_H
#define FLOW_SENSOR_H

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define FLOW_EDGE_QUEUE_LEN 32
#define FLOW_MIN_PERIOD_US INT64_C(1000)

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

// initialize rising-edge interrupts and timestamp queue
void flow_sensor_init(void);

// accepted pulse total, independent of timestamp queue capacity
int flow_sensor_get_count(flow_channel_t channel);

// timestamp queue one entry per pulse edge
QueueHandle_t flow_sensor_get_edge_queue(void);

#endif
