/**
 * Sensor measurement task (one-shot with deep sleep).
 * On each boot: reads VL53L0X distance + battery ADC,
 * reports via Zigbee, then enters deep sleep until next interval.
 */

#include "sensor_task.h"
#include "vl53l0x.h"
#include "zigbee_device.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sensor_task";

#define BATTERY_ADC_CHANNEL         ADC_CHANNEL_2  /* GPIO2/A0 on XIAO ESP32-C6 - battery (200k PCB + 100k soldered) */
#define BATTERY_ADC_ATTEN           ADC_ATTEN_DB_12
#define BATTERY_VOLTAGE_MULTIPLIER  2.86f   /* calibrated: 200k (PCB) + 100k (soldered) */

#define DEEP_SLEEP_MIN_US           55000000ULL  /* 55s minimum fallback */
#define POST_REPORT_WAIT_MS         3000         /* wait for ACK + Z2M command window */
#define OTA_CHECK_WAIT_MS           50000        /* wait for Z2M to respond to OTA query (~35s observed) */
#define OTA_TRANSFER_TIMEOUT_US     600000000ULL /* 10 min OTA transfer timeout */

static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t adc_cali_handle = NULL;

static esp_err_t adc_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &adc_handle);
    if (err != ESP_OK) return err;

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = BATTERY_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(adc_handle, BATTERY_ADC_CHANNEL, &chan_cfg);
    if (err != ESP_OK) return err;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .chan = BATTERY_ADC_CHANNEL,
        .atten = BATTERY_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_cali_create_scheme_curve_fitting(&cali_cfg, &adc_cali_handle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = BATTERY_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_cali_create_scheme_line_fitting(&cali_cfg, &adc_cali_handle);
#endif
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ADC calibration not available, using raw values");
        adc_cali_handle = NULL;
    }

    return ESP_OK;
}

static float read_battery_voltage(void)
{
    int raw = 0;
    esp_err_t err = adc_oneshot_read(adc_handle, BATTERY_ADC_CHANNEL, &raw);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ADC read failed: %s", esp_err_to_name(err));
        return 0.0f;
    }

    float voltage;
    if (adc_cali_handle) {
        int mv = 0;
        adc_cali_raw_to_voltage(adc_cali_handle, raw, &mv);
        voltage = (float)mv / 1000.0f;
    } else {
        voltage = (float)raw / 4095.0f * 3.3f;
    }

    voltage *= BATTERY_VOLTAGE_MULTIPLIER;
    return voltage;
}

static void sensor_task_fn(void *pvParameters)
{
    int64_t t_start = esp_timer_get_time();

    ESP_LOGI(TAG, "=== Measurement ===");

    /* Initialize ADC (re-init required after every deep sleep wakeup) */
    esp_err_t err = adc_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC init failed: %s", esp_err_to_name(err));
    }

    /* Brief delay for Zigbee stack to stabilize after join/rejoin */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* Power on TOF, measure, power off */
    int distance_mm = -1;
    err = vl53l0x_power_on();
    if (err == ESP_OK) {
        distance_mm = vl53l0x_read_distance_mm();
        vl53l0x_power_off();
    } else {
        ESP_LOGE(TAG, "TOF power on failed: %s", esp_err_to_name(err));
    }

    /* Read battery voltage */
    float battery_v = read_battery_voltage();

    /* Calculate fill level if distance valid */
    float fill_pct = -1.0f;
    if (distance_mm >= 0) {
        uint32_t tank_h = zigbee_get_tank_height();
        if (tank_h > 0 && (uint32_t)distance_mm <= tank_h) {
            fill_pct = (float)(tank_h - (uint32_t)distance_mm) / (float)tank_h * 100.0f;
            if (fill_pct < 0.0f) fill_pct = 0.0f;
            if (fill_pct > 100.0f) fill_pct = 100.0f;
        } else {
            fill_pct = 0.0f;
        }
    }

    /* Report to Zigbee coordinator */
    if (distance_mm >= 0) {
        ESP_LOGI(TAG, "Distance: %d mm | Fill: %.1f%% | Battery: %.2f V", distance_mm, fill_pct, battery_v);
        zigbee_report_value(EP_DISTANCE, (float)distance_mm);
        zigbee_report_value(EP_FILL_LEVEL, fill_pct);
    } else {
        ESP_LOGW(TAG, "Distance: OUT OF RANGE | Battery: %.2f V", battery_v);
    }
    zigbee_report_value(EP_BATTERY, battery_v);

    /* Wait for report ACK + give Z2M window to send commands (e.g. interval change) */
    vTaskDelay(pdMS_TO_TICKS(POST_REPORT_WAIT_MS));

    /* Wait for OTA query to fire (OTA client sends Query Next Image Request shortly after join) */
    int64_t ota_check_start = esp_timer_get_time();
    while (!zigbee_ota_in_progress() &&
           (esp_timer_get_time() - ota_check_start) < (int64_t)OTA_CHECK_WAIT_MS * 1000LL) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    /* If OTA started, stay awake until it finishes */
    if (zigbee_ota_in_progress()) {
        ESP_LOGI(TAG, "OTA in progress - staying awake...");
        int64_t ota_start = esp_timer_get_time();
        while (zigbee_ota_in_progress()) {
            vTaskDelay(pdMS_TO_TICKS(2000));
            if ((esp_timer_get_time() - ota_start) > (int64_t)OTA_TRANSFER_TIMEOUT_US) {
                ESP_LOGW(TAG, "OTA transfer timeout - going to sleep");
                break;
            }
        }
        ESP_LOGI(TAG, "OTA done or timeout");
    }

    /* Read interval just before sleeping — picks up any Z2M writes during wake window */
    uint32_t interval_min = zigbee_get_measurement_interval();

    /* Calculate how long to sleep */
    int64_t elapsed_us = esp_timer_get_time() - t_start;
    uint64_t interval_us = (uint64_t)interval_min * 60ULL * 1000000ULL;
    uint64_t sleep_us;
    if (interval_us > (uint64_t)elapsed_us + 5000000ULL) {
        sleep_us = interval_us - (uint64_t)elapsed_us;
    } else {
        sleep_us = DEEP_SLEEP_MIN_US;
    }

    ESP_LOGI(TAG, "Deep sleep for %llu s (target %lu min, elapsed %lld ms)",
             sleep_us / 1000000ULL, (unsigned long)interval_min, elapsed_us / 1000LL);

    esp_sleep_enable_timer_wakeup(sleep_us);
    esp_deep_sleep_start();
    /* Never reached */
}

void sensor_task_start(void)
{
    xTaskCreate(sensor_task_fn, "sensor_task", 4096, NULL, 5, NULL);
}
