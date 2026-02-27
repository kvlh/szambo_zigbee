/**
 * Sensor measurement task.
 * Periodically reads VL53L0X distance and battery ADC,
 * then reports values to Zigbee coordinator.
 */

#include "sensor_task.h"
#include "vl53l0x.h"
#include "zigbee_device.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sensor_task";

/* Interval is dynamic, read from zigbee_get_measurement_interval() */
#define BATTERY_ADC_CHANNEL         ADC_CHANNEL_2  /* GPIO2/A0 on XIAO ESP32-C6 - battery (needs 200k to GND) */
#define BATTERY_ADC_ATTEN           ADC_ATTEN_DB_12
#define BATTERY_VOLTAGE_MULTIPLIER  2.86f   /* calibrated: 200k (PCB) + 100k (soldered), actual ratio */

static TaskHandle_t sensor_task_handle = NULL;
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

    /* Try to create calibration handle */
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

    ESP_LOGI(TAG, "ADC initialized for battery measurement");
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
        /* Rough conversion without calibration (12-bit, 12dB ~0-3.3V) */
        voltage = (float)raw / 4095.0f * 3.3f;
    }

    voltage *= BATTERY_VOLTAGE_MULTIPLIER;
    return voltage;
}

static void sensor_task_fn(void *pvParameters)
{
    uint32_t interval_min = zigbee_get_measurement_interval();
    ESP_LOGI(TAG, "Sensor task started, interval=%lu min", (unsigned long)interval_min);

    /* Initialize ADC */
    esp_err_t err = adc_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC init failed: %s", esp_err_to_name(err));
    }

    /* Initial delay to let Zigbee stabilize */
    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1) {
        /* Re-read interval each cycle (may have been changed via Z2M) */
        interval_min = zigbee_get_measurement_interval();
        uint32_t interval_ms = interval_min * 60U * 1000U;

        ESP_LOGI(TAG, "--- Measurement cycle (interval=%lu min) ---", (unsigned long)interval_min);

        /* Power on TOF sensor */
        err = vl53l0x_power_on();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "TOF power on failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(interval_ms));
            continue;
        }

        /* Read distance */
        int distance_mm = vl53l0x_read_distance_mm();

        /* Power off TOF sensor */
        vl53l0x_power_off();

        /* Read battery voltage */
        float battery_v = read_battery_voltage();

        /* Report to Zigbee */
        if (distance_mm >= 0) {
            float dist_f = (float)distance_mm;
            ESP_LOGI(TAG, "Distance: %d mm | Battery: %.2f V", distance_mm, battery_v);
            zigbee_report_value(EP_DISTANCE, dist_f);
        } else {
            ESP_LOGW(TAG, "Distance: OUT OF RANGE | Battery: %.2f V", battery_v);
        }
        zigbee_report_value(EP_BATTERY, battery_v);

        ESP_LOGI(TAG, "Sleeping for %lu min (light sleep enabled)", (unsigned long)interval_min);
        vTaskDelay(pdMS_TO_TICKS(interval_ms));
    }
}

void sensor_task_start(void)
{
    if (sensor_task_handle != NULL) {
        ESP_LOGW(TAG, "Sensor task already running");
        return;
    }

    xTaskCreate(sensor_task_fn, "sensor_task", 4096, NULL, 5, &sensor_task_handle);
}
