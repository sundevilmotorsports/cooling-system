#ifndef BOARD_H
#define BOARD_H

#include "driver/gpio.h"

// coolant flow sensor
#define FLOW1_GPIO GPIO_NUM_2 // 2 on c5
#define FLOW2_GPIO GPIO_NUM_1 // 1 on c5

// i2c
#define I2C_SDA_GPIO GPIO_NUM_7 // 16 on c5
#define I2C_SCL_GPIO GPIO_NUM_8 // 15 on c5

// twai
#define CAN1_TX GPIO_NUM_5 // 18 on c5
#define CAN1_RX GPIO_NUM_6 // 17 on c5

#endif
