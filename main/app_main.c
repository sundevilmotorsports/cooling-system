#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h> 

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_twai_types.h"

#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"

#include "board.h"
#include "can_tx.h"
#include "flow_sensor.h"
#include "flow_calc.h"
#include "ads1115.h"
#include "temp_calc.h"

#define STACK_SIZE 4096
#define FLOW_PROCESSOR_PRIORITY 5
#define ADC_SAMPLER_PRIORITY 4
#define TIMEOUT_US (int64_t)(3 * 1e6) // 3 seconds
#define ADC_ROUND_ROBIN_DELAY_MS 500
#define FLOW_LOG_INTERVAL_US INT64_C(1000000) // 1 second

// calculation state for each channel
static flow_calc_state_t flow_state[2];
static const float lpp = 390.0f; // pulses per liter

// processes data from queue
void process_flow(void *arg) {
    flow_edge_event_t event;
    int counts[2] = {0, 0};
    float rates[2] = {0.0f, 0.0f};
    float volumes[2] = {0.0f, 0.0f};
    int64_t next_log_us = esp_timer_get_time() + FLOW_LOG_INTERVAL_US;

    while (1) {
        // block up to 100ms for an edge, so timeouts still get checked with no flow
        if (xQueueReceive(flow_sensor_get_edge_queue(), &event, pdMS_TO_TICKS(100)) == pdTRUE) {
            flow_calc_process_edge(&flow_state[event.channel], event.timestamp_us, lpp, FLOW_MIN_PERIOD_US);
        }

        int64_t now_us = esp_timer_get_time();

        // publish channels
        for (flow_channel_t channel = FLOW_CHANNEL_1; channel <= FLOW_CHANNEL_2; channel++) {
            flow_calc_check_timeout(&flow_state[channel], now_us, TIMEOUT_US);

            counts[channel] = flow_sensor_get_count(channel);
            rates[channel] = flow_calc_get_rate(&flow_state[channel]);
            volumes[channel] = flow_calc_get_volume(counts[channel], lpp);
            update_flow(channel, rates[channel], volumes[channel]);
        }

        // Log from the task, never the ISR.
        if (now_us >= next_log_us) {
            flow_channel_t channel = FLOW_CHANNEL_1;
            ESP_LOGI("flow", "ch%d: pulses=%d rate=%.2f L/min total=%.3f L",
                     channel + 1, counts[channel], rates[channel], volumes[channel]);
            next_log_us = now_us + FLOW_LOG_INTERVAL_US;
        }
    }
}

// Analog1-4 are NTC thermistors, mapped to temp_c_1-4.
void process_adc(void *arg) {
    while (1) {
        for (uint8_t channel = 0; channel < ADS1115_NUM_CHANNELS; channel++) {
            int16_t raw = 0;
            esp_err_t err = ads1115_read_channel(channel, &raw);
            if (err != ESP_OK) {
                update_temp(channel, NAN);
                if (channel < 2) {
                    ESP_LOGW("adc_sampler", "Analog%d read failed: %s", channel + 1, esp_err_to_name(err));
                }
                continue;
            }
            float volts = temp_calc_volts(raw);
            float ohms = temp_calc_ntc_ohms(volts, TEMP_CALC_R_BIAS_OHMS, TEMP_CALC_V_SUPPLY);
            float temp_c = temp_calc_ntc_c(ohms);
            update_temp(channel, temp_c);
            if (channel >= 2) {
                continue;
            }
            if (isfinite(temp_c)) {
                ESP_LOGI("adc_sampler", "Analog%d: %d counts, %.3f V, %.2f C", channel + 1, raw, volts, temp_c);
            } else {
                ESP_LOGW("adc_sampler", "Analog%d: invalid NTC reading (%d counts, %.3f V)", channel + 1, raw, volts);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(ADC_ROUND_ROBIN_DELAY_MS));
    }
}

void app_main() {
    flow_sensor_init();
    ads1115_init();
    can_init();   

    flow_calc_state_init(&flow_state[0]);
    flow_calc_state_init(&flow_state[1]);

    // create tasks
    xTaskCreate(process_flow, "flow_processor", STACK_SIZE, NULL, FLOW_PROCESSOR_PRIORITY, NULL);
    xTaskCreate(process_adc, "adc_sampler", STACK_SIZE, NULL, ADC_SAMPLER_PRIORITY, NULL);
}
