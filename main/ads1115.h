#ifndef ADS1115_H
#define ADS1115_H

#include <stdint.h>

#include "esp_err.h"

#define ADS1115_NUM_CHANNELS 4

// installs i2c bus + device
void ads1115_init(void);

// reads one channel (0-3) into *out as a raw signed 16-bit ADC count
// returns ESP_OK, or an I2C/timeout error the caller must handle - a dead
// +3.3VA rail must not take down the node
esp_err_t ads1115_read_channel(uint8_t channel, int16_t *out);

#endif
