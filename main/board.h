#ifndef BOARD_H
#define BOARD_H

#include "driver/gpio.h"

// coolant flow sensor
#define FLOW1_GPIO GPIO_NUM_9
#define FLOW2_GPIO GPIO_NUM_8

// i2c
#define I2C_SCL_GPIO GPIO_NUM_2
#define I2C_SDA_GPIO GPIO_NUM_1

// twai
#define CAN1_TX GPIO_NUM_43
#define CAN1_RX GPIO_NUM_44

#endif
