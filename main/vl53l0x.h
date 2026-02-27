#pragma once

#include "esp_err.h"
#include "driver/i2c.h"

#define VL53L0X_I2C_ADDR        0x29
#define VL53L0X_I2C_PORT        I2C_NUM_0
#define VL53L0X_SDA_GPIO        16
#define VL53L0X_SCL_GPIO        17
#define VL53L0X_SHUT_GPIO       23
#define VL53L0X_I2C_FREQ_HZ    100000

/**
 * Initialize I2C bus and SHUT pin.
 * Does NOT power on the sensor yet.
 */
esp_err_t vl53l0x_init(void);

/**
 * Power on sensor (SHUT=LOW), wait for boot, run setup sequence.
 */
esp_err_t vl53l0x_power_on(void);

/**
 * Power off sensor (SHUT=HIGH).
 */
void vl53l0x_power_off(void);

/**
 * Perform a single-shot distance measurement.
 * Sensor must be powered on first.
 * Returns distance in millimeters, or -1 on error/out-of-range.
 */
int vl53l0x_read_distance_mm(void);
