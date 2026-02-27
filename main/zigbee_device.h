#pragma once

#include "esp_zigbee_core.h"

/* Device identity */
#define MANUFACTURER_NAME       "esphome"
#define MODEL_IDENTIFIER        "szambo_tof_sensor"

/* Endpoints */
#define EP_DISTANCE             1   /* genAnalogInput: distance in mm */
#define EP_BATTERY              2   /* genAnalogInput: battery voltage */
#define EP_CONFIG               3   /* genAnalogOutput: measurement interval */

/* Measurement interval config */
#define DEFAULT_MEASUREMENT_INTERVAL_MIN    60
#define NVS_NAMESPACE           "szambo"
#define NVS_KEY_INTERVAL        "interval"

/* OTA identifiers */
#define OTA_UPGRADE_MANUFACTURER    0x1001
#define OTA_UPGRADE_IMAGE_TYPE      0x1011
#define OTA_UPGRADE_FILE_VERSION    0x01000001  /* 1.0.0.1 */
#define OTA_UPGRADE_HW_VERSION      1
#define OTA_UPGRADE_MAX_DATA_SIZE   64

/* Zigbee config */
#define INSTALLCODE_POLICY_ENABLE   false
#define ED_AGING_TIMEOUT            ESP_ZB_ED_AGING_TIMEOUT_16384MIN
#define ED_KEEP_ALIVE               3000    /* ms */

/**
 * Initialize and start Zigbee stack (call from app_main).
 * This creates endpoints, registers signal handler, and starts the stack.
 */
void zigbee_task(void *pvParameters);

/**
 * Report sensor value on an endpoint.
 * @param endpoint EP_DISTANCE or EP_BATTERY
 * @param value    sensor reading (float)
 */
void zigbee_report_value(uint8_t endpoint, float value);

/**
 * Set measurement interval (minutes). Saves to NVS.
 * @param minutes interval in minutes, clamped to [1, 1440]
 */
void zigbee_set_measurement_interval(uint32_t minutes);

/**
 * Get current measurement interval in minutes.
 */
uint32_t zigbee_get_measurement_interval(void);

