#ifndef ADS1115_H
#define ADS1115_H

#include <stdint.h>

#define ADS1115_NUM_CHANNELS 4

// installs i2c bus + device
void ads1115_init(void);

// returns raw signed 16-bit ADC count for one channel (0-3)
int16_t ads1115_read_channel(uint8_t channel);

#endif
