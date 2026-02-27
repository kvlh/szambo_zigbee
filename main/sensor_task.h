#pragma once

/**
 * Start the sensor measurement task.
 * Call after successfully joining the Zigbee network.
 * Creates a FreeRTOS task that periodically reads VL53L0X and ADC,
 * then reports values via Zigbee.
 */
void sensor_task_start(void);
