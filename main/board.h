#ifndef BOARD_H
#define BOARD_H

#include "driver/gpio.h"

// coolant flow sensor
#define FLOW1_GPIO GPIO_NUM_2 // 1 on c3 super mini
#define FLOW2_GPIO GPIO_NUM_1 // 0 on c3 super mini

// i2c
#define I2C_SDA_GPIO GPIO_NUM_7 // 7 on c3 super mini
#define I2C_SCL_GPIO GPIO_NUM_8 // 8 on c3 super mini

// twai
#define CAN1_TX GPIO_NUM_5 // 5 on c3 super mini
#define CAN1_RX GPIO_NUM_6 // 6 on c3 super mini

#endif
