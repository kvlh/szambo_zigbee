/**
 * VL53L0X I2C driver for ESP-IDF
 * Ported from ESPHome vl53l0x_sensor.cpp (Pololu/ST reference)
 * Long-range mode enabled by default.
 */

#include "vl53l0x.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "vl53l0x";

static uint8_t stop_variable;

/* ---------- low-level I2C helpers ---------- */

static esp_err_t vl53_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_write_to_device(VL53L0X_I2C_PORT, VL53L0X_I2C_ADDR,
                                      buf, 2, pdMS_TO_TICKS(100));
}

static esp_err_t vl53_write_reg16(uint8_t reg, uint16_t val)
{
    uint8_t buf[3] = {reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF)};
    return i2c_master_write_to_device(VL53L0X_I2C_PORT, VL53L0X_I2C_ADDR,
                                      buf, 3, pdMS_TO_TICKS(100));
}

static uint8_t vl53_read_reg(uint8_t reg)
{
    uint8_t val = 0;
    i2c_master_write_read_device(VL53L0X_I2C_PORT, VL53L0X_I2C_ADDR,
                                 &reg, 1, &val, 1, pdMS_TO_TICKS(100));
    return val;
}

static uint16_t vl53_read_reg16(uint8_t reg)
{
    uint8_t buf[2] = {0};
    i2c_master_write_read_device(VL53L0X_I2C_PORT, VL53L0X_I2C_ADDR,
                                 &reg, 1, buf, 2, pdMS_TO_TICKS(100));
    return ((uint16_t)buf[0] << 8) | buf[1];
}

static esp_err_t vl53_write_bytes(uint8_t reg, const uint8_t *data, size_t len)
{
    uint8_t buf[1 + len];
    buf[0] = reg;
    memcpy(&buf[1], data, len);
    return i2c_master_write_to_device(VL53L0X_I2C_PORT, VL53L0X_I2C_ADDR,
                                      buf, 1 + len, pdMS_TO_TICKS(100));
}

static esp_err_t vl53_read_bytes(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(VL53L0X_I2C_PORT, VL53L0X_I2C_ADDR,
                                        &reg, 1, data, len, pdMS_TO_TICKS(100));
}

/* bitwise helpers */
static void vl53_set_bits(uint8_t reg, uint8_t mask)
{
    uint8_t val = vl53_read_reg(reg);
    vl53_write_reg(reg, val | mask);
}

static void vl53_clear_bits(uint8_t reg, uint8_t mask)
{
    uint8_t val = vl53_read_reg(reg);
    vl53_write_reg(reg, val & ~mask);
}

/* ---------- timing budget helpers ---------- */

typedef struct {
    bool tcc, msrc, dss, pre_range, final_range;
} seq_step_enables_t;

typedef struct {
    uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
    uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
    uint32_t msrc_dss_tcc_us, pre_range_us, final_range_us;
} seq_step_timeouts_t;

static uint8_t get_vcsel_pulse_period(int type)
{
    uint8_t vcsel;
    if (type == 0)      vcsel = vl53_read_reg(0x50); /* pre-range */
    else if (type == 1) vcsel = vl53_read_reg(0x70); /* final-range */
    else return 255;
    return (vcsel + 1) << 1;
}

static uint32_t get_macro_period(uint8_t vcsel_period_pclks)
{
    return ((2304UL * vcsel_period_pclks * 1655UL) + 500UL) / 1000UL;
}

static uint32_t timeout_mclks_to_us(uint16_t mclks, uint8_t vcsel_period_pclks)
{
    uint32_t macro_ns = get_macro_period(vcsel_period_pclks);
    return ((mclks * macro_ns) + (macro_ns / 2)) / 1000;
}

static uint32_t timeout_us_to_mclks(uint32_t us, uint8_t vcsel_period_pclks)
{
    uint32_t macro_ns = get_macro_period(vcsel_period_pclks);
    return (((us * 1000) + (macro_ns / 2)) / macro_ns);
}

static uint16_t decode_timeout(uint16_t reg_val)
{
    uint8_t msb = (reg_val >> 8) & 0xFF;
    uint8_t lsb = reg_val & 0xFF;
    return ((uint16_t)lsb << msb) + 1;
}

static uint16_t encode_timeout(uint16_t timeout_mclks)
{
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;
    if (timeout_mclks <= 0) return 0;
    ls_byte = timeout_mclks - 1;
    while ((ls_byte & 0xFFFFFF00) > 0) {
        ls_byte >>= 1;
        ms_byte++;
    }
    return (ms_byte << 8) | (ls_byte & 0xFF);
}

static void get_seq_step_enables(seq_step_enables_t *en)
{
    uint8_t sc = vl53_read_reg(0x01);
    en->tcc         = (sc >> 4) & 1;
    en->dss         = (sc >> 3) & 1;
    en->msrc        = (sc >> 2) & 1;
    en->pre_range   = (sc >> 6) & 1;
    en->final_range = (sc >> 7) & 1;
}

static void get_seq_step_timeouts(const seq_step_enables_t *en, seq_step_timeouts_t *to)
{
    to->pre_range_vcsel_period_pclks = get_vcsel_pulse_period(0);
    to->msrc_dss_tcc_mclks = vl53_read_reg(0x46) + 1;
    to->msrc_dss_tcc_us = timeout_mclks_to_us(to->msrc_dss_tcc_mclks, to->pre_range_vcsel_period_pclks);

    to->pre_range_mclks = decode_timeout(vl53_read_reg16(0x51));
    to->pre_range_us = timeout_mclks_to_us(to->pre_range_mclks, to->pre_range_vcsel_period_pclks);

    to->final_range_vcsel_period_pclks = get_vcsel_pulse_period(1);
    to->final_range_mclks = decode_timeout(vl53_read_reg16(0x71));
    if (en->pre_range) {
        to->final_range_mclks -= to->pre_range_mclks;
    }
    to->final_range_us = timeout_mclks_to_us(to->final_range_mclks, to->final_range_vcsel_period_pclks);
}

static uint32_t get_timing_budget(void)
{
    seq_step_enables_t en;
    seq_step_timeouts_t to;
    uint32_t budget = 1910 + 960;
    get_seq_step_enables(&en);
    get_seq_step_timeouts(&en, &to);
    if (en.tcc) budget += to.msrc_dss_tcc_us + 590;
    if (en.dss) budget += 2 * (to.msrc_dss_tcc_us + 690);
    else if (en.msrc) budget += to.msrc_dss_tcc_us + 660;
    if (en.pre_range) budget += to.pre_range_us + 660;
    if (en.final_range) budget += to.final_range_us + 550;
    return budget;
}

static bool set_timing_budget(uint32_t budget_us)
{
    seq_step_enables_t en;
    seq_step_timeouts_t to;
    if (budget_us < 20000) return false;

    uint32_t used = 1320 + 960;
    get_seq_step_enables(&en);
    get_seq_step_timeouts(&en, &to);

    if (en.tcc) used += to.msrc_dss_tcc_us + 590;
    if (en.dss) used += 2 * (to.msrc_dss_tcc_us + 690);
    else if (en.msrc) used += to.msrc_dss_tcc_us + 660;
    if (en.pre_range) used += to.pre_range_us + 660;

    if (en.final_range) {
        used += 550;
        if (used > budget_us) return false;
        uint32_t final_timeout_us = budget_us - used;
        uint16_t final_mclks = timeout_us_to_mclks(final_timeout_us, to.final_range_vcsel_period_pclks);
        if (en.pre_range) final_mclks += to.pre_range_mclks;
        vl53_write_reg16(0x71, encode_timeout(final_mclks));
    }
    return true;
}

static bool perform_single_ref_calibration(uint8_t vhv_init_byte)
{
    vl53_write_reg(0x00, 0x01 | vhv_init_byte);
    uint32_t start = xTaskGetTickCount();
    while ((vl53_read_reg(0x13) & 0x07) == 0) {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(1000))
            return false;
        vTaskDelay(1);
    }
    vl53_write_reg(0x0B, 0x01);
    vl53_write_reg(0x00, 0x00);
    return true;
}

/* ---------- public API ---------- */

esp_err_t vl53l0x_init(void)
{
    /* Configure SHUT pin */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << VL53L0X_SHUT_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(VL53L0X_SHUT_GPIO, 1); /* start powered off */

    /* Configure I2C */
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = VL53L0X_SDA_GPIO,
        .scl_io_num = VL53L0X_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = VL53L0X_I2C_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(VL53L0X_I2C_PORT, &i2c_conf);
    if (err != ESP_OK) return err;

    err = i2c_driver_install(VL53L0X_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "I2C and SHUT pin initialized");
    return ESP_OK;
}

esp_err_t vl53l0x_power_on(void)
{
    ESP_LOGI(TAG, "Powering on sensor...");
    gpio_set_level(VL53L0X_SHUT_GPIO, 0); /* SHUT=LOW -> sensor ON */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* Setup sequence - ported from ESPHome (long_range = true) */
    vl53_set_bits(0x89, 0x01);
    vl53_write_reg(0x88, 0x00);

    vl53_write_reg(0x80, 0x01);
    vl53_write_reg(0xFF, 0x01);
    vl53_write_reg(0x00, 0x00);
    stop_variable = vl53_read_reg(0x91);
    vl53_write_reg(0x00, 0x01);
    vl53_write_reg(0xFF, 0x00);
    vl53_write_reg(0x80, 0x00);
    vl53_set_bits(0x60, 0x12);

    /* Signal rate limit for long range: 0.1 * 128 = 12.8 -> 13 */
    vl53_write_reg16(0x44, 13);
    vl53_write_reg(0x01, 0xFF);

    /* getSpadInfo */
    vl53_write_reg(0x80, 0x01);
    vl53_write_reg(0xFF, 0x01);
    vl53_write_reg(0x00, 0x00);
    vl53_write_reg(0xFF, 0x06);
    vl53_set_bits(0x83, 0x04);
    vl53_write_reg(0xFF, 0x07);
    vl53_write_reg(0x81, 0x01);
    vl53_write_reg(0x80, 0x01);
    vl53_write_reg(0x94, 0x6B);
    vl53_write_reg(0x83, 0x00);

    uint32_t start = xTaskGetTickCount();
    while (vl53_read_reg(0x83) == 0x00) {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(1000)) {
            ESP_LOGE(TAG, "SPAD info timeout");
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(1);
    }

    vl53_write_reg(0x83, 0x01);
    uint8_t tmp = vl53_read_reg(0x92);
    uint8_t spad_count = tmp & 0x7F;
    bool spad_type_is_aperture = tmp & 0x80;

    vl53_write_reg(0x81, 0x00);
    vl53_write_reg(0xFF, 0x06);
    vl53_clear_bits(0x83, 0x04);
    vl53_write_reg(0xFF, 0x01);
    vl53_write_reg(0x00, 0x01);
    vl53_write_reg(0xFF, 0x00);
    vl53_write_reg(0x80, 0x00);

    uint8_t ref_spad_map[6] = {0};
    vl53_read_bytes(0xB0, ref_spad_map, 6);

    vl53_write_reg(0xFF, 0x01);
    vl53_write_reg(0x4F, 0x00);
    vl53_write_reg(0x4E, 0x2C);
    vl53_write_reg(0xFF, 0x00);
    vl53_write_reg(0xB6, 0xB4);

    uint8_t first_spad = spad_type_is_aperture ? 12 : 0;
    uint8_t spads_enabled = 0;
    for (int i = 0; i < 48; i++) {
        uint8_t *val = &ref_spad_map[i / 8];
        uint8_t mask = 1 << (i % 8);
        if (i < first_spad || spads_enabled == spad_count) {
            *val &= ~mask;
        } else if (*val & mask) {
            spads_enabled++;
        }
    }
    vl53_write_bytes(0xB0, ref_spad_map, 6);

    /* Default tuning (long_range variant) */
    vl53_write_reg(0xFF, 0x01); vl53_write_reg(0x00, 0x00);
    vl53_write_reg(0xFF, 0x00); vl53_write_reg(0x09, 0x00);
    vl53_write_reg(0x10, 0x00); vl53_write_reg(0x11, 0x00);
    vl53_write_reg(0x24, 0x01); vl53_write_reg(0x25, 0xFF);
    vl53_write_reg(0x75, 0x00);
    vl53_write_reg(0xFF, 0x01); vl53_write_reg(0x4E, 0x2C);
    vl53_write_reg(0x48, 0x00); vl53_write_reg(0x30, 0x20);
    vl53_write_reg(0xFF, 0x00);
    vl53_write_reg(0x30, 0x07); /* long range */
    vl53_write_reg(0x54, 0x00); vl53_write_reg(0x31, 0x04);
    vl53_write_reg(0x32, 0x03); vl53_write_reg(0x40, 0x83);
    vl53_write_reg(0x46, 0x25); vl53_write_reg(0x60, 0x00);
    vl53_write_reg(0x27, 0x00); vl53_write_reg(0x50, 0x06);
    vl53_write_reg(0x51, 0x00); vl53_write_reg(0x52, 0x96);
    vl53_write_reg(0x56, 0x08);
    vl53_write_reg(0x57, 0x50); /* long range */
    vl53_write_reg(0x61, 0x00); vl53_write_reg(0x62, 0x00);
    vl53_write_reg(0x64, 0x00); vl53_write_reg(0x65, 0x00);
    vl53_write_reg(0x66, 0xA0);
    vl53_write_reg(0xFF, 0x01); vl53_write_reg(0x22, 0x32);
    vl53_write_reg(0x47, 0x14); vl53_write_reg(0x49, 0xFF);
    vl53_write_reg(0x4A, 0x00);
    vl53_write_reg(0xFF, 0x00); vl53_write_reg(0x7A, 0x0A);
    vl53_write_reg(0x7B, 0x00); vl53_write_reg(0x78, 0x21);
    vl53_write_reg(0xFF, 0x01); vl53_write_reg(0x23, 0x34);
    vl53_write_reg(0x42, 0x00); vl53_write_reg(0x44, 0xFF);
    vl53_write_reg(0x45, 0x26); vl53_write_reg(0x46, 0x05);
    vl53_write_reg(0x40, 0x40); vl53_write_reg(0x0E, 0x06);
    vl53_write_reg(0x20, 0x1A); vl53_write_reg(0x43, 0x40);
    vl53_write_reg(0xFF, 0x00); vl53_write_reg(0x34, 0x03);
    vl53_write_reg(0x35, 0x44);
    vl53_write_reg(0xFF, 0x01); vl53_write_reg(0x31, 0x04);
    vl53_write_reg(0x4B, 0x09); vl53_write_reg(0x4C, 0x05);
    vl53_write_reg(0x4D, 0x04);
    vl53_write_reg(0xFF, 0x00); vl53_write_reg(0x44, 0x00);
    vl53_write_reg(0x45, 0x20); vl53_write_reg(0x47, 0x08);
    vl53_write_reg(0x48, 0x48); /* long range */
    vl53_write_reg(0x67, 0x00); vl53_write_reg(0x70, 0x04);
    vl53_write_reg(0x71, 0x01); vl53_write_reg(0x72, 0xFE);
    vl53_write_reg(0x76, 0x00); vl53_write_reg(0x77, 0x00);
    vl53_write_reg(0xFF, 0x01); vl53_write_reg(0x0D, 0x01);
    vl53_write_reg(0xFF, 0x00); vl53_write_reg(0x80, 0x01);
    vl53_write_reg(0x01, 0xF8);
    vl53_write_reg(0xFF, 0x01); vl53_write_reg(0x8E, 0x01);
    vl53_write_reg(0x00, 0x01);
    vl53_write_reg(0xFF, 0x00); vl53_write_reg(0x80, 0x00);

    vl53_write_reg(0x0A, 0x04);
    vl53_clear_bits(0x84, 0x10);
    vl53_write_reg(0x0B, 0x01);

    /* Set timing budget */
    uint32_t budget = get_timing_budget();
    vl53_write_reg(0x01, 0xE8);
    set_timing_budget(budget);
    vl53_write_reg(0x01, 0x01);

    /* Reference calibrations */
    if (!perform_single_ref_calibration(0x40)) {
        ESP_LOGE(TAG, "1st ref calibration failed");
        return ESP_FAIL;
    }
    vl53_write_reg(0x01, 0x02);
    if (!perform_single_ref_calibration(0x00)) {
        ESP_LOGE(TAG, "2nd ref calibration failed");
        return ESP_FAIL;
    }
    vl53_write_reg(0x01, 0xE8);

    ESP_LOGI(TAG, "Sensor setup complete (long-range mode, SPAD count=%d)", spad_count);
    return ESP_OK;
}

void vl53l0x_power_off(void)
{
    gpio_set_level(VL53L0X_SHUT_GPIO, 1); /* SHUT=HIGH -> sensor OFF */
    ESP_LOGD(TAG, "Sensor powered off");
}

int vl53l0x_read_distance_mm(void)
{
    /* Initiate single-shot measurement */
    vl53_write_reg(0x80, 0x01);
    vl53_write_reg(0xFF, 0x01);
    vl53_write_reg(0x00, 0x00);
    vl53_write_reg(0x91, stop_variable);
    vl53_write_reg(0x00, 0x01);
    vl53_write_reg(0xFF, 0x00);
    vl53_write_reg(0x80, 0x00);
    vl53_write_reg(0x00, 0x01);

    /* Wait for measurement start */
    uint32_t start = xTaskGetTickCount();
    while (vl53_read_reg(0x00) & 0x01) {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(1000)) {
            ESP_LOGE(TAG, "Measurement start timeout");
            return -1;
        }
        vTaskDelay(1);
    }

    /* Wait for measurement complete */
    start = xTaskGetTickCount();
    while ((vl53_read_reg(0x13) & 0x07) == 0) {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(1000)) {
            ESP_LOGE(TAG, "Measurement complete timeout");
            return -1;
        }
        vTaskDelay(1);
    }

    uint16_t range_mm = vl53_read_reg16(0x1E); /* 0x14 + 10 = 0x1E */
    vl53_write_reg(0x0B, 0x01);

    if (range_mm >= 8190) {
        ESP_LOGW(TAG, "Out of range (>8190mm)");
        return -1;
    }

    ESP_LOGD(TAG, "Distance: %u mm", range_mm);
    return (int)range_mm;
}
