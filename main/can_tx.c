#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_twai_types.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include "can_tx.h"
#include "board.h"

twai_node_handle_t CAN1 = NULL;
static const char *TAG = "cooling system";

static flow_data_t s_data;
static portMUX_TYPE s_data_mux = portMUX_INITIALIZER_UNLOCKED; // prevents reading partial writes

// writes current state of flow
void update_flow(flow_channel_t channel, float rate_lpm, float total_vol) {
    taskENTER_CRITICAL(&s_data_mux);
    s_data.rate_lpm[channel] = rate_lpm;
    s_data.total_volume_l[channel] = total_vol;
    taskEXIT_CRITICAL(&s_data_mux);
}

// gets the current flow state
void get_flow(flow_data_t *out) {
    taskENTER_CRITICAL(&s_data_mux);
    *out = s_data;
    taskEXIT_CRITICAL(&s_data_mux);
}

// initializes twai can node
void can_init() {
    twai_onchip_node_config_t node_config = {
        .io_cfg.tx = CAN1_TX,
        .io_cfg.rx = CAN1_RX,
        .bit_timing.bitrate = 200000,
        .tx_queue_depth = 5,
    };

    esp_err_t ret = twai_new_node_onchip(&node_config, &CAN1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create CAN node: %s", esp_err_to_name(ret));
        return;
    }

    ret = twai_node_enable(CAN1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable CAN node: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "CAN initialized on TX=%d, RX=%d @ 200kbps", CAN1_TX, CAN1_RX);
}