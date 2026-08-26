#include "driver/i2c_master.h"
#include "esp_log.h"

#include "board.h"
#include "ads1115.h"

#define ADS1115_ADDR 0x48
#define ADS1115_REG_CONVERSION 0x00
#define ADS1115_REG_CONFIG 0x01

// config register fields (datasheet section 9.6.3)
#define ADS1115_OS_START (1u << 15)
#define ADS1115_MUX_AIN0 (4u) // shift per channel in read_channel
#define ADS1115_PGA_4_096V (1u << 9)
#define ADS1115_MODE_SINGLE_SHOT (1u << 8)
#define ADS1115_DR_8SPS (0u << 5)
#define ADS1115_COMP_QUE_DISABLE (3u)

static const char *TAG = "ads1115";

static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_dev = NULL;

void ads1115_init(void) {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = -1,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false, // external pull-up
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADS1115_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev));

    ESP_LOGI(TAG, "ADS1115 init on SDA=%d, SCL=%d", I2C_SDA_GPIO, I2C_SCL_GPIO);
}

int16_t ads1115_read_channel(uint8_t channel) {
    uint16_t config = ADS1115_OS_START |
                       ((ADS1115_MUX_AIN0 + channel) << 12) |
                       ADS1115_PGA_4_096V |
                       ADS1115_MODE_SINGLE_SHOT |
                       ADS1115_DR_8SPS |
                       ADS1115_COMP_QUE_DISABLE;

    // start conversion
    uint8_t buffer[3] = {
        ADS1115_REG_CONFIG,
        (uint8_t)(config >> 8),
        (uint8_t)(config & 0xFF),
    };
    ESP_ERROR_CHECK(i2c_master_transmit(s_dev, buffer, sizeof(buffer), -1));

    // poll config register until OS bit goes back to 1
    uint8_t reg = ADS1115_REG_CONFIG;
    uint8_t status[2];
    do {
        ESP_ERROR_CHECK(i2c_master_transmit_receive(s_dev, &reg, 1, status, sizeof(status), -1));
    } while (!(status[0] & 0x80));

    // read
    reg = ADS1115_REG_CONVERSION;
    uint8_t raw[2];
    ESP_ERROR_CHECK(i2c_master_transmit_receive(s_dev, &reg, 1, raw, sizeof(raw), -1));

    return (int16_t)((raw[0] << 8) | raw[1]);
}
