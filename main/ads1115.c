#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

// bounded waits
#define ADS1115_I2C_TIMEOUT_MS 100
#define ADS1115_POLL_INTERVAL_MS 10
#define ADS1115_CONVERSION_TIMEOUT_MS 250 // 8 SPS is 125ms/conversion

static const char *TAG = "ads1115";

static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_dev = NULL;

static void i2c_pin_check(void) {
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << I2C_SDA_GPIO) | (1ULL << I2C_SCL_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    esp_rom_delay_us(1000);
    int sda = gpio_get_level(I2C_SDA_GPIO), scl = gpio_get_level(I2C_SCL_GPIO);

    // internal pull-up proves the pin itself can be driven high
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&cfg));
    esp_rom_delay_us(1000);
    int sda_pu = gpio_get_level(I2C_SDA_GPIO), scl_pu = gpio_get_level(I2C_SCL_GPIO);

    // hand the pins back floating for the i2c driver
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&cfg));

    ESP_LOGI(TAG, "pins: idle SDA=%d SCL=%d | int pullup SDA=%d SCL=%d",
             sda, scl, sda_pu, scl_pu);
    if (sda && scl) ESP_LOGI(TAG, "both lines idle high on external pullups");
    if (!scl && scl_pu) ESP_LOGW(TAG, "SCL floats low without internal pullup; check external pullup and continuity to ADS1115");
    else if (!scl_pu) ESP_LOGE(TAG, "SCL stays low with internal pullup; check for a short or wrong GPIO");
    if (!sda && sda_pu) ESP_LOGW(TAG, "SDA floats low without internal pullup; check external pullup and continuity to ADS1115");
    else if (!sda_pu) ESP_LOGE(TAG, "SDA stays low with internal pullup; check for a short or wrong GPIO");
}

void ads1115_init(void) {
    i2c_pin_check();

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

    esp_err_t err = i2c_master_probe(s_bus, ADS1115_ADDR, 20);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "ADS1115 acknowledged at 0x%02X", ADS1115_ADDR);
    } else {
        ESP_LOGE(TAG, "no ACK from ADS1115 at 0x%02X: %s", ADS1115_ADDR, esp_err_to_name(err));
    }
}

esp_err_t ads1115_read_channel(uint8_t channel, int16_t *out) {
    if (channel >= ADS1115_NUM_CHANNELS || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

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
    esp_err_t err = i2c_master_transmit(s_dev, buffer, sizeof(buffer), ADS1115_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }

    // poll config register until OS bit goes back to 1, bounded by conversion time
    uint8_t reg = ADS1115_REG_CONFIG;
    uint8_t status[2];
    int64_t deadline_us = esp_timer_get_time() + (int64_t)ADS1115_CONVERSION_TIMEOUT_MS * 1000;

    while (1) {
        // delay before polling
        vTaskDelay(pdMS_TO_TICKS(ADS1115_POLL_INTERVAL_MS));

        err = i2c_master_transmit_receive(s_dev, &reg, 1, status, sizeof(status), ADS1115_I2C_TIMEOUT_MS);
        if (err != ESP_OK) {
            return err;
        }
        // conversion complete
        if (status[0] & 0x80) {
            break;
        }
        // took too long
        if (esp_timer_get_time() > deadline_us) {
            return ESP_ERR_TIMEOUT;
        }
    }

    // read
    reg = ADS1115_REG_CONVERSION;
    uint8_t raw[2];
    err = i2c_master_transmit_receive(s_dev, &reg, 1, raw, sizeof(raw), ADS1115_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }

    *out = (int16_t)((raw[0] << 8) | raw[1]);
    return ESP_OK;
}
