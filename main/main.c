/**
 * Szambo TOF Native Firmware
 * ESP32-C6 + VL53L0X + Zigbee + OTA
 *
 * Main entry point: initializes NVS, power management, VL53L0X hardware,
 * and starts Zigbee task.
 */

#include "vl53l0x.h"
#include "zigbee_device.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_zigbee_core.h"
#include "platform/esp_zigbee_platform.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#ifdef CONFIG_PM_ENABLE
#include "esp_pm.h"
#include "esp_sleep.h"
#endif

static const char *TAG = "main";

static esp_err_t esp_zb_power_save_init(void)
{
    esp_err_t rc = ESP_OK;
#ifdef CONFIG_PM_ENABLE
    int cur_cpu_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    esp_pm_config_t pm_config = {
        .max_freq_mhz = cur_cpu_freq_mhz,
        .min_freq_mhz = cur_cpu_freq_mhz,
        /* Start with light sleep DISABLED; enabled after Zigbee commissioning */
        .light_sleep_enable = false,
    };
    rc = esp_pm_configure(&pm_config);
#endif
    return rc;
}

void enable_pm_light_sleep(void)
{
#ifdef CONFIG_PM_ENABLE
    int cur_cpu_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    esp_pm_config_t pm_config = {
        .max_freq_mhz = cur_cpu_freq_mhz,
        .min_freq_mhz = cur_cpu_freq_mhz,
        .light_sleep_enable = true,
    };
    esp_pm_configure(&pm_config);
    ESP_LOGI(TAG, "PM light sleep enabled");
#endif
}

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Szambo TOF Sensor - Native Zigbee + OTA");
    ESP_LOGI(TAG, "========================================");

    /* Initialize NVS */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS: erasing and reinitializing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* Initialize VL53L0X I2C and SHUT pin (sensor stays off until measurement) */
    err = vl53l0x_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "VL53L0X init failed: %s", esp_err_to_name(err));
        /* Continue anyway - Zigbee should still work */
    }

    /* Initialize Zigbee light sleep power management */
    ESP_ERROR_CHECK(esp_zb_power_save_init());

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
