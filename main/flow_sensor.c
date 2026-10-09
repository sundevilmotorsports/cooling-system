#include <limits.h>

#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "board.h"
#include "flow_sensor.h"

static QueueHandle_t s_edge_queue = NULL;
static int s_pulse_count[2] = {0, 0};
static int64_t s_last_edge_us[2] = {-1, -1};
static portMUX_TYPE s_count_mux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR flow_edge_isr(void *arg) {
    flow_channel_t channel = (flow_channel_t)(intptr_t)arg;
    int64_t now_us = esp_timer_get_time();

    // Rejected edges must not advance the reference timestamp.
    if (s_last_edge_us[channel] != -1 &&
        now_us - s_last_edge_us[channel] < FLOW_MIN_PERIOD_US) {
        return;
    }
    s_last_edge_us[channel] = now_us;

    taskENTER_CRITICAL_ISR(&s_count_mux);
    // shortcut: saturate at INT_MAX; widen the count API for longer deployments.
    if (s_pulse_count[channel] < INT_MAX) {
        s_pulse_count[channel]++;
    }
    taskEXIT_CRITICAL_ISR(&s_count_mux);

    flow_edge_event_t event = {
        .channel = channel,
        .timestamp_us = now_us,
    };
    BaseType_t woken = pdFALSE;
    // Count first so a full timestamp queue cannot lose volume.
    xQueueSendFromISR(s_edge_queue, &event, &woken);
    if (woken) {
        portYIELD_FROM_ISR();
    }
}

void flow_sensor_init(void) {
    s_edge_queue = xQueueCreate(FLOW_EDGE_QUEUE_LEN, sizeof(flow_edge_event_t));
    ESP_ERROR_CHECK(s_edge_queue != NULL ? ESP_OK : ESP_ERR_NO_MEM);

    gpio_config_t flow_gpio_cfg = {
        .pin_bit_mask = (1ULL << FLOW1_GPIO) | (1ULL << FLOW2_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&flow_gpio_cfg));
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK(gpio_isr_handler_add(FLOW1_GPIO, flow_edge_isr, (void *)FLOW_CHANNEL_1));
    ESP_ERROR_CHECK(gpio_isr_handler_add(FLOW2_GPIO, flow_edge_isr, (void *)FLOW_CHANNEL_2));
}

int flow_sensor_get_count(flow_channel_t channel) {
    taskENTER_CRITICAL(&s_count_mux);
    int count = s_pulse_count[channel];
    taskEXIT_CRITICAL(&s_count_mux);
    return count;
}

QueueHandle_t flow_sensor_get_edge_queue(void) {
    return s_edge_queue;
}
