#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_twai_types.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"

#include "board.h"
#include "can_tx.h"
#include "flow_sensor.h"
#include "flow_calc.h"

#define STACK_SIZE 4096
#define FLOW_PROCESSOR_PRIORITY 5
#define MIN_PERIOD_US (int64_t)15000  
#define TIMEOUT_US (int64_t)(3 * 1e6) // 3 seconds

// calc state for each channel
static flow_calc_state_t flow_state[2];
static const float lpp = 20.0f; // pulses per liter 

// processes data from queeue
void process_flow(void *arg) {
    flow_edge_event_t event;

    while (1) {
        // block up to 100ms for an edge, so timeouts still get checked with no flow
        if (xQueueReceive(flow_sensor_get_edge_queue(), &event, pdMS_TO_TICKS(100)) == pdTRUE) {
            flow_calc_process_edge(&flow_state[event.channel], event.timestamp_us, lpp, MIN_PERIOD_US);
            
            // flow_sensor_get_count() includes overflow accumulation (flag_accum_count=1)
            float total_vol = flow_calc_get_volume(flow_sensor_get_count(event.channel), lpp);
            update_flow(event.channel, flow_calc_get_rate(&flow_state[event.channel]), total_vol);
        }

        flow_calc_check_timeout(&flow_state[0], esp_timer_get_time(), TIMEOUT_US);
        flow_calc_check_timeout(&flow_state[1], esp_timer_get_time(), TIMEOUT_US);
    }
}

void app_main() {
    can_init();
    flow_sensor_init();

    flow_calc_state_init(&flow_state[0]);
    flow_calc_state_init(&flow_state[1]);

    // create task, no need for handle as nothing else interacts with it
    xTaskCreate(process_flow, "flow_processor", STACK_SIZE, NULL, FLOW_PROCESSOR_PRIORITY, NULL);
}
