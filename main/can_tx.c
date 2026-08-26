#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_twai_types.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#include "can_tx.h"
#include "cooling_system.h"
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

// packs and transmits one message
static void can_tx_send(uint32_t frame_id, uint8_t *buf, size_t len) {
    twai_frame_t frame = {
        .header.id = frame_id,
        .header.dlc = len,
        .buffer = buf,
        .buffer_len = len,
    };

    esp_err_t ret = twai_node_transmit(CAN1, &frame, 0);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "TX 0x%lx failed: %s", (unsigned long)frame_id, esp_err_to_name(ret));
    }
}

// fires every FLOW cycle time - 100ms for now
static void can_tx_timer_cb(void *arg) {
    // get current flow data and write
    flow_data_t data;
    get_flow(&data);

    // create CAN structs, pack into buffers, and transmit
    struct cooling_system_flow1_t flow1 = {
        .rate_lpm_1 = cooling_system_flow1_rate_lpm_1_encode(data.rate_lpm[FLOW_CHANNEL_1]),
        .total_volume_l_1 = cooling_system_flow1_total_volume_l_1_encode(data.total_volume_l[FLOW_CHANNEL_1]),
    };
    uint8_t buf1[COOLING_SYSTEM_FLOW1_LENGTH];
    cooling_system_flow1_pack(buf1, &flow1, sizeof(buf1));
    can_tx_send(COOLING_SYSTEM_FLOW1_FRAME_ID, buf1, sizeof(buf1));

    struct cooling_system_flow2_t flow2 = {
        .rate_lpm_2 = cooling_system_flow2_rate_lpm_2_encode(data.rate_lpm[FLOW_CHANNEL_2]),
        .total_volume_l_2 = cooling_system_flow2_total_volume_l_2_encode(data.total_volume_l[FLOW_CHANNEL_2]),
    };
    uint8_t buf2[COOLING_SYSTEM_FLOW2_LENGTH];
    cooling_system_flow2_pack(buf2, &flow2, sizeof(buf2));
    can_tx_send(COOLING_SYSTEM_FLOW2_FRAME_ID, buf2, sizeof(buf2));
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

    // timer - transmit data every 100ms
    const esp_timer_create_args_t timer_args = {
        .callback = can_tx_timer_cb,
        .name = "can_tx",
    };
    esp_timer_handle_t timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, COOLING_SYSTEM_FLOW1_CYCLE_TIME_MS * 1000ULL));
}