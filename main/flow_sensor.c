#include "driver/pulse_cnt.h"
#include "hal/pcnt_types.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "board.h"
#include "flow_sensor.h"

// pcnt units store pulse count - volume
// xQueue stores timestamps with channel - timing

// retained handles for pcnt units
static pcnt_unit_handle_t s_pcnt_unit[2] = {NULL, NULL};

// edge timestamps queue handle
static QueueHandle_t s_edge_queue = NULL;

// overflow count to verify flag_accum_count (per unit)
static volatile uint32_t s_overflow_count[2] = {0, 0};

// increments overflow counter when limit is hit 
static bool IRAM_ATTR pcnt_watch_cb(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) {
    flow_channel_t channel = (flow_channel_t)(intptr_t)user_ctx;
    s_overflow_count[channel]++;
    return false; // no higher-priority task to wake
}

// runs on each pulse - sends edge event to queue
static void IRAM_ATTR flow_edge_isr(void *arg) {
    flow_edge_event_t event = {
        .channel = (flow_channel_t)(intptr_t)arg,
        .timestamp_us = esp_timer_get_time(),
    };

    BaseType_t woken = pdFALSE;

    xQueueSendFromISR(s_edge_queue, &event, &woken);

    if (woken) {
        portYIELD_FROM_ISR();
    }
}

// installing pcnt unit (pulse counter)
void flow_sensor_init() {
    // sets high and low limits - resets to 0 after next pulse
    // highest possible vals for continuous flow tracking
    pcnt_unit_config_t unit_cfg = { 
        .low_limit =  -32768,  // backward flow
        .high_limit = 32767,   // forward flow
        .flag_accum_count = 1  // accumulates count after resetting due to limit
    };

    // create new units - two flow channels
    pcnt_unit_handle_t pcnt_unit_1 = NULL, pcnt_unit_2 = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &pcnt_unit_1));
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &pcnt_unit_2));

    // set handles
    s_pcnt_unit[FLOW_CHANNEL_1] = pcnt_unit_1;
    s_pcnt_unit[FLOW_CHANNEL_2] = pcnt_unit_2;

    // channels - one per unit
    pcnt_chan_config_t chan_config_1 = {
        .edge_gpio_num = FLOW1_GPIO,
        .level_gpio_num = -1, // not using level
    };

    pcnt_chan_config_t chan_config_2 = {
        .edge_gpio_num = FLOW2_GPIO,
        .level_gpio_num = -1,
    };

    pcnt_channel_handle_t pcnt_chan_1 = NULL, pcnt_chan_2 = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_1, &chan_config_1, &pcnt_chan_1));
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_2, &chan_config_2, &pcnt_chan_2));

    // glitch filter - any pulse duration shorter is treated as noise - doesnt increment counter
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1500,
    };

    pcnt_unit_set_glitch_filter(pcnt_unit_1, &filter_config);
    pcnt_unit_set_glitch_filter(pcnt_unit_2, &filter_config);

    // set actions on edge - counter increases on rising edge, holds on falling edge
    pcnt_channel_set_edge_action(pcnt_chan_1, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    pcnt_channel_set_edge_action(pcnt_chan_2, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);

    // watch points to track when pulse count surpasses limits
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit_1, 32767));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit_1, -32768));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit_2, 32767));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit_2, -32768));

    // overflow verification
    pcnt_event_callbacks_t watch_cbs = { .on_reach = pcnt_watch_cb  };
    
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit_1, &watch_cbs, (void *)FLOW_CHANNEL_1));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit_2, &watch_cbs, (void *)FLOW_CHANNEL_2));

    // enable and start units
    pcnt_unit_enable(pcnt_unit_1);
    pcnt_unit_clear_count(pcnt_unit_1);
    pcnt_unit_start(pcnt_unit_1);

    pcnt_unit_enable(pcnt_unit_2);
    pcnt_unit_clear_count(pcnt_unit_2);
    pcnt_unit_start(pcnt_unit_2);

    // edge timestamp queue
    s_edge_queue = xQueueCreate(FLOW_EDGE_QUEUE_LEN, sizeof(flow_edge_event_t));

    gpio_config_t flow_gpio_cfg = {
        .pin_bit_mask = (1ULL << FLOW1_GPIO) | (1ULL << FLOW2_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };

    ESP_ERROR_CHECK(gpio_config(&flow_gpio_cfg));

    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    // ensures functions run when triggered on specific gpios
    ESP_ERROR_CHECK(gpio_isr_handler_add(FLOW1_GPIO, flow_edge_isr, (void *)FLOW_CHANNEL_1));
    ESP_ERROR_CHECK(gpio_isr_handler_add(FLOW2_GPIO, flow_edge_isr, (void *)FLOW_CHANNEL_2));
}

// get pulse count
int flow_sensor_get_count(flow_channel_t channel) {
    int count = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(s_pcnt_unit[channel], &count));
    return count;
}

// overflow count (amt of times pcnt unit surpassed limit) - verify against flag_accum_count
uint32_t flow_sensor_get_overflow_count(flow_channel_t channel) {
    return s_overflow_count[channel];
}

QueueHandle_t flow_sensor_get_edge_queue(void) {
    return s_edge_queue;
}