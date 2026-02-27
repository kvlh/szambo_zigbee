/**
 * Szambo TOF Native Firmware
 * ESP32-C6 + VL53L0X + Zigbee + OTA + Deep Sleep
 *
 * Each boot: join/rejoin Zigbee → measure → report → deep sleep for interval.
 */

#include "vl53l0x.h"
#include "zigbee_device.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "esp_zigbee_core.h"
#include "platform/esp_zigbee_platform.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Szambo TOF Sensor - Native Zigbee + OTA");
    ESP_LOGI(TAG, "========================================");

    /* Log wakeup cause for diagnostics */
    esp_sleep_wakeup_cause_t wakeup = esp_sleep_get_wakeup_cause();
    if (wakeup == ESP_SLEEP_WAKEUP_TIMER) {
        ESP_LOGI(TAG, "Wakeup: deep sleep timer");
    } else {
        ESP_LOGI(TAG, "Wakeup: power-on / reset (cause=%d)", wakeup);
    }

    /* Initialize NVS */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS: erasing and reinitializing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* Initialize VL53L0X I2C and SHUT pin */
    err = vl53l0x_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "VL53L0X init failed: %s", esp_err_to_name(err));
    }

    /* Configure Zigbee platform (required for ESP-IDF v5.3+) */
    esp_zb_platform_config_t config = {
        .radio_config = {
            .radio_mode = ZB_RADIO_MODE_NATIVE,
        },
        .host_config = {
            .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,
        },
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    /* Start Zigbee task */
    xTaskCreate(zigbee_task, "zigbee_task", 4096, NULL, 5, NULL);
}
