/**
 * Zigbee device implementation with OTA support.
 * Two endpoints with genAnalogInput cluster for distance and battery.
 * OTA client cluster on EP1 for firmware updates via Z2M.
 */

#include "zigbee_device.h"
#include "sensor_task.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_zigbee_core.h"
#include "esp_zigbee_ota.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "zb_device";

/* Measurement interval (minutes), loaded from NVS on boot */
static uint32_t measurement_interval_min = DEFAULT_MEASUREMENT_INTERVAL_MIN;

/* Tank height (mm), loaded from NVS on boot */
static uint32_t tank_height_mm = DEFAULT_TANK_HEIGHT_MM;

/* OTA state */
static volatile bool s_ota_in_progress = false;
static esp_ota_handle_t ota_handle = 0;
static const esp_partition_t *ota_partition = NULL;
static uint32_t ota_total_size = 0;
static uint32_t ota_received = 0;

/* ZCL char string: first byte is length, then chars (no null terminator) */
/* ZCL char string: first byte is length, followed by the characters */
static uint8_t manufacturer_name[] = {7, 'e', 's', 'p', 'h', 'o', 'm', 'e'};
static uint8_t model_identifier[]  = {17, 's', 'z', 'a', 'm', 'b', 'o', '_', 't', 'o', 'f', '_', 's', 'e', 'n', 's', 'o', 'r'};

/* ---------- Cluster list creation ---------- */

static esp_zb_cluster_list_t *create_analog_input_endpoint_clusters(const char *description)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    /* Basic cluster */
    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x03, /* battery */
    };
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
                                  manufacturer_name);
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
                                  model_identifier);
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* Identify cluster */
    esp_zb_identify_cluster_cfg_t identify_cfg = {
        .identify_time = 0,
    };
    esp_zb_attribute_list_t *identify_cluster = esp_zb_identify_cluster_create(&identify_cfg);
    esp_zb_cluster_list_add_identify_cluster(cluster_list, identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* Analog Input cluster */
    esp_zb_analog_input_cluster_cfg_t analog_cfg = {
        .out_of_service = false,
        .present_value = 0.0f,
        .status_flags = 0,
    };
    esp_zb_attribute_list_t *analog_cluster = esp_zb_analog_input_cluster_create(&analog_cfg);
    /* Add description attribute */
    esp_zb_cluster_add_attr(analog_cluster, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
                            ESP_ZB_ZCL_ATTR_ANALOG_INPUT_DESCRIPTION_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                            (void *)description);
    esp_zb_cluster_list_add_analog_input_cluster(cluster_list, analog_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    return cluster_list;
}

static void add_ota_client_cluster(esp_zb_cluster_list_t *cluster_list)
{
    esp_zb_ota_cluster_cfg_t ota_cfg = {
        .ota_upgrade_file_version = OTA_UPGRADE_FILE_VERSION,
        .ota_upgrade_manufacturer = OTA_UPGRADE_MANUFACTURER,
        .ota_upgrade_image_type = OTA_UPGRADE_IMAGE_TYPE,
    };
    esp_zb_attribute_list_t *ota_cluster = esp_zb_ota_cluster_create(&ota_cfg);

    /* Set timer_query=1 min so QueryNextImageRequest fires within the OTA wake window.
     * Default SDK init sets timer_counter=1440 (24h). Setting the client variable
     * attribute (0xfff1) overrides this at cluster creation time (safe, not runtime). */
    esp_zb_zcl_ota_upgrade_client_variable_t ota_var = {
        .timer_query  = 1,
        .hw_version   = OTA_UPGRADE_HW_VERSION,
        .max_data_size = OTA_UPGRADE_MAX_DATA_SIZE,
    };
    esp_zb_ota_cluster_add_attr(ota_cluster, 0xfff1, &ota_var);

    esp_zb_cluster_list_add_ota_cluster(cluster_list, ota_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
}

/* ---------- NVS helpers for measurement interval ---------- */

static void nvs_load_interval(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        uint32_t val = 0;
        if (nvs_get_u32(nvs, NVS_KEY_INTERVAL, &val) == ESP_OK && val >= 1 && val <= 1440) {
            measurement_interval_min = val;
        }
        nvs_close(nvs);
    }
    ESP_LOGI(TAG, "Measurement interval: %lu min", (unsigned long)measurement_interval_min);
}

static void nvs_save_interval(uint32_t minutes)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_u32(nvs, NVS_KEY_INTERVAL, minutes);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

void zigbee_set_measurement_interval(uint32_t minutes)
{
    if (minutes < 1) minutes = 1;
    if (minutes > 1440) minutes = 1440;
    measurement_interval_min = minutes;
    nvs_save_interval(minutes);
    ESP_LOGI(TAG, "Measurement interval set to %lu min", (unsigned long)minutes);
}

uint32_t zigbee_get_measurement_interval(void)
{
    return measurement_interval_min;
}

/* ---------- NVS helpers for tank height ---------- */

static void nvs_load_tank_height(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        uint32_t val = 0;
        if (nvs_get_u32(nvs, NVS_KEY_TANK_HEIGHT, &val) == ESP_OK && val >= 100 && val <= 10000) {
            tank_height_mm = val;
        }
        nvs_close(nvs);
    }
    ESP_LOGI(TAG, "Tank height: %lu mm", (unsigned long)tank_height_mm);
}

static void nvs_save_tank_height(uint32_t mm)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_u32(nvs, NVS_KEY_TANK_HEIGHT, mm);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

void zigbee_set_tank_height(uint32_t mm)
{
    if (mm < 100) mm = 100;
    if (mm > 10000) mm = 10000;
    tank_height_mm = mm;
    nvs_save_tank_height(mm);
    ESP_LOGI(TAG, "Tank height set to %lu mm", (unsigned long)mm);
}

uint32_t zigbee_get_tank_height(void)
{
    return tank_height_mm;
}

bool zigbee_ota_in_progress(void)
{
    return s_ota_in_progress;
}

/* ---------- EP3: config endpoint (genAnalogOutput) ---------- */

static uint8_t ao_description[] = {24, 'm', 'e', 'a', 's', 'u', 'r', 'e', 'm', 'e', 'n', 't', '_', 'i', 'n', 't', 'e', 'r', 'v', 'a', 'l', '_', 'm', 'i', 'n'};
static uint8_t ao_tank_description[] = {14, 't', 'a', 'n', 'k', '_', 'h', 'e', 'i', 'g', 'h', 't', '_', 'm', 'm'};

static esp_zb_cluster_list_t *create_config_endpoint_clusters(void)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    /* Basic cluster */
    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x03,
    };
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
                                  manufacturer_name);
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
                                  model_identifier);
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* Identify cluster */
    esp_zb_identify_cluster_cfg_t identify_cfg = { .identify_time = 0 };
    esp_zb_attribute_list_t *identify_cluster = esp_zb_identify_cluster_create(&identify_cfg);
    esp_zb_cluster_list_add_identify_cluster(cluster_list, identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* Analog Output cluster */
    esp_zb_analog_output_cluster_cfg_t ao_cfg = {
        .out_of_service = false,
        .present_value = (float)measurement_interval_min,
        .status_flags = 0,
    };
    esp_zb_attribute_list_t *ao_cluster = esp_zb_analog_output_cluster_create(&ao_cfg);

    /* Add description attribute */
    esp_zb_cluster_add_attr(ao_cluster, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT,
                            ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_DESCRIPTION_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                            (void *)ao_description);

    /* Add min/max present value attributes */
    float min_val = 1.0f;
    float max_val = 1440.0f;
    esp_zb_cluster_add_attr(ao_cluster, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT,
                            ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_MIN_PRESENT_VALUE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_SINGLE,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                            &min_val);
    esp_zb_cluster_add_attr(ao_cluster, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT,
                            ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_MAX_PRESENT_VALUE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_SINGLE,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                            &max_val);

    esp_zb_cluster_list_add_analog_output_cluster(cluster_list, ao_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    return cluster_list;
}

static esp_zb_cluster_list_t *create_tank_height_endpoint_clusters(void)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    /* Basic cluster */
    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x03,
    };
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
                                  manufacturer_name);
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
                                  model_identifier);
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* Identify cluster */
    esp_zb_identify_cluster_cfg_t identify_cfg = { .identify_time = 0 };
    esp_zb_attribute_list_t *identify_cluster = esp_zb_identify_cluster_create(&identify_cfg);
    esp_zb_cluster_list_add_identify_cluster(cluster_list, identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* Analog Output cluster */
    esp_zb_analog_output_cluster_cfg_t ao_cfg = {
        .out_of_service = false,
        .present_value = (float)tank_height_mm,
        .status_flags = 0,
    };
    esp_zb_attribute_list_t *ao_cluster = esp_zb_analog_output_cluster_create(&ao_cfg);

    /* Add description attribute */
    esp_zb_cluster_add_attr(ao_cluster, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT,
                            ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_DESCRIPTION_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                            (void *)ao_tank_description);

    /* Add min/max present value attributes */
    float min_val = 100.0f;
    float max_val = 10000.0f;
    esp_zb_cluster_add_attr(ao_cluster, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT,
                            ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_MIN_PRESENT_VALUE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_SINGLE,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                            &min_val);
    esp_zb_cluster_add_attr(ao_cluster, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT,
                            ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_MAX_PRESENT_VALUE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_SINGLE,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                            &max_val);

    esp_zb_cluster_list_add_analog_output_cluster(cluster_list, ao_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    return cluster_list;
}

/* ---------- SET_ATTR_VALUE callback ---------- */

static esp_err_t zb_set_attr_value_handler(esp_zb_zcl_set_attr_value_message_t message)
{
    if (message.info.status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "Set attr value status: 0x%x", message.info.status);
        return ESP_ERR_INVALID_STATE;
    }

    if (message.info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT &&
        message.attribute.id == ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID) {
        float val = *(float *)message.attribute.data.value;

        if (message.info.dst_endpoint == EP_CONFIG) {
            uint32_t minutes = (uint32_t)val;
            if (minutes < 1) minutes = 1;
            if (minutes > 1440) minutes = 1440;
            zigbee_set_measurement_interval(minutes);
            float clamped = (float)minutes;
            esp_zb_zcl_set_attribute_val(EP_CONFIG,
                ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID,
                &clamped, false);
        } else if (message.info.dst_endpoint == EP_TANK_HEIGHT) {
            uint32_t mm = (uint32_t)val;
            if (mm < 100) mm = 100;
            if (mm > 10000) mm = 10000;
            zigbee_set_tank_height(mm);
            float clamped = (float)mm;
            esp_zb_zcl_set_attribute_val(EP_TANK_HEIGHT,
                ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID,
                &clamped, false);
        }
    }

    return ESP_OK;
}

/* ---------- OTA callbacks ---------- */

static esp_err_t zb_ota_upgrade_status_handler(esp_zb_zcl_ota_upgrade_value_message_t message)
{
    if (message.info.status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "OTA status error: 0x%x", message.info.status);
        return ESP_FAIL;
    }

    switch (message.upgrade_status) {
    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_START:
        ESP_LOGI(TAG, "===== OTA START: total size %lu =====", (unsigned long)message.ota_header.image_size);
        s_ota_in_progress = true;
        ota_total_size = message.ota_header.image_size;
        ota_received = 0;
        ota_partition = esp_ota_get_next_update_partition(NULL);
        if (!ota_partition) {
            ESP_LOGE(TAG, "No OTA partition available");
            return ESP_FAIL;
        }
        esp_err_t err = esp_ota_begin(ota_partition, OTA_SIZE_UNKNOWN, &ota_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "OTA partition: %s", ota_partition->label);
        break;

    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_RECEIVE: {
        ota_received += message.payload_size;
        ESP_LOGI(TAG, "OTA receive: %lu / %lu bytes (%lu%%)",
                 (unsigned long)ota_received, (unsigned long)ota_total_size,
                 ota_total_size ? (unsigned long)(ota_received * 100 / ota_total_size) : 0);
        esp_err_t err = esp_ota_write(ota_handle, message.payload, message.payload_size);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            return ESP_FAIL;
        }
        break;
    }

    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_APPLY:
        ESP_LOGI(TAG, "OTA APPLY - finalizing...");
        break;

    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_CHECK:
        ESP_LOGI(TAG, "OTA CHECK - version verification");
        break;

    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_FINISH: {
        ESP_LOGI(TAG, "===== OTA FINISH =====");
        esp_err_t err = esp_ota_end(ota_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
            return ESP_FAIL;
        }
        err = esp_ota_set_boot_partition(ota_partition);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "OTA complete, rebooting in 3s...");
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart();
        break;
    }

    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_ABORT:
        ESP_LOGW(TAG, "OTA ABORTED");
        s_ota_in_progress = false;
        if (ota_handle) {
            esp_ota_abort(ota_handle);
            ota_handle = 0;
        }
        break;

    default:
        ESP_LOGW(TAG, "OTA unknown status: %d", message.upgrade_status);
        break;
    }

    return ESP_OK;
}

static esp_err_t zb_ota_upgrade_query_image_handler(esp_zb_zcl_ota_upgrade_query_image_resp_message_t message)
{
    if (message.info.status == ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "OTA query image response: version=0x%08lx, size=%lu",
                 (unsigned long)message.file_version, (unsigned long)message.image_size);
        /* OTA upgrade proceeds automatically via the stack after query response */
    } else {
        ESP_LOGI(TAG, "OTA: No image available (status=0x%x)", message.info.status);
    }
    return ESP_OK;
}

/* ---------- Zigbee signal handler ---------- */

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialized");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Device started (first_start=%d)",
                     sig_type == ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START);
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Factory new - starting steering...");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Already commissioned - starting sensor task");
                sensor_task_start();
            }
        } else {
            ESP_LOGW(TAG, "Init failed (status=%d), retrying...", err_status);
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t ext_pan_id;
            esp_zb_get_extended_pan_id(ext_pan_id);
            ESP_LOGI(TAG, "==== JOINED NETWORK ====");
            ESP_LOGI(TAG, "PAN ID: 0x%04hx, Channel: %d",
                     esp_zb_get_pan_id(), esp_zb_get_current_channel());
            ESP_LOGI(TAG, "Extended PAN: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
                     ext_pan_id[7], ext_pan_id[6], ext_pan_id[5], ext_pan_id[4],
                     ext_pan_id[3], ext_pan_id[2], ext_pan_id[1], ext_pan_id[0]);
            sensor_task_start();
        } else {
            ESP_LOGW(TAG, "Steering failed (status=%d), retrying in 1s...", err_status);
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        }
        break;

    default:
        ESP_LOGD(TAG, "ZDO signal: %s (0x%x), status: %s",
                 esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

/* ---------- Action handler ---------- */

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        return zb_set_attr_value_handler(*(esp_zb_zcl_set_attr_value_message_t *)message);
    case ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID:
        return zb_ota_upgrade_status_handler(*(esp_zb_zcl_ota_upgrade_value_message_t *)message);
    case ESP_ZB_CORE_OTA_UPGRADE_QUERY_IMAGE_RESP_CB_ID:
        return zb_ota_upgrade_query_image_handler(*(esp_zb_zcl_ota_upgrade_query_image_resp_message_t *)message);
    default:
        ESP_LOGD(TAG, "Unhandled action callback: 0x%x", callback_id);
        break;
    }
    return ESP_OK;
}

/* ---------- Report sensor value ---------- */

void zigbee_report_value(uint8_t endpoint, float value)
{
    /* Acquire Zigbee stack lock (required in v1.6+ when calling from non-Zigbee task) */
    esp_zb_lock_acquire(portMAX_DELAY);

    /* Set the presentValue attribute */
    esp_zb_zcl_status_t status = esp_zb_zcl_set_attribute_val(
        endpoint,
        ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID,
        &value,
        false);

    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "Failed to set attribute on EP%d: 0x%x", endpoint, status);
        esp_zb_lock_release();
        return;
    }

    /* Send report to coordinator (explicit short address 0x0000) */
    esp_zb_zcl_report_attr_cmd_t report = {0};
    report.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
    report.zcl_basic_cmd.dst_endpoint = 1;
    report.zcl_basic_cmd.src_endpoint = endpoint;
    report.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    report.clusterID = ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT;
    report.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    report.attributeID = ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID;
    esp_zb_zcl_report_attr_cmd_req(&report);

    esp_zb_lock_release();
    ESP_LOGD(TAG, "Reported EP%d value: %.2f", endpoint, value);
}

/* ---------- Zigbee task ---------- */

void zigbee_task(void *pvParameters)
{
    /* Load config from NVS */
    nvs_load_interval();
    nvs_load_tank_height();

    /* Zigbee stack config - end device */
    esp_zb_cfg_t zb_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,
        .nwk_cfg.zed_cfg = {
            .ed_timeout = ED_AGING_TIMEOUT,
            .keep_alive = ED_KEEP_ALIVE,
        },
    };
    esp_zb_init(&zb_cfg);

    /* Create EP1: distance sensor + OTA client */
    esp_zb_cluster_list_t *ep1_clusters = create_analog_input_endpoint_clusters("distance_mm");
    add_ota_client_cluster(ep1_clusters);

    esp_zb_endpoint_config_t ep1_cfg = {
        .endpoint = EP_DISTANCE,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_ep_list_add_ep(ep_list, ep1_clusters, ep1_cfg);

    /* Create EP2: battery voltage */
    esp_zb_cluster_list_t *ep2_clusters = create_analog_input_endpoint_clusters("battery_voltage");

    esp_zb_endpoint_config_t ep2_cfg = {
        .endpoint = EP_BATTERY,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, ep2_clusters, ep2_cfg);

    /* Create EP3: config (measurement interval) */
    esp_zb_cluster_list_t *ep3_clusters = create_config_endpoint_clusters();

    esp_zb_endpoint_config_t ep3_cfg = {
        .endpoint = EP_CONFIG,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, ep3_clusters, ep3_cfg);

    /* Create EP4: fill level (read-only, calculated) */
    esp_zb_cluster_list_t *ep4_clusters = create_analog_input_endpoint_clusters("fill_level_pct");

    esp_zb_endpoint_config_t ep4_cfg = {
        .endpoint = EP_FILL_LEVEL,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, ep4_clusters, ep4_cfg);

    /* Create EP5: tank height (configurable) */
    esp_zb_cluster_list_t *ep5_clusters = create_tank_height_endpoint_clusters();

    esp_zb_endpoint_config_t ep5_cfg = {
        .endpoint = EP_TANK_HEIGHT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, ep5_clusters, ep5_cfg);

    /* Register device */
    esp_zb_device_register(ep_list);

    /* Register action handler for OTA callbacks */
    esp_zb_core_action_handler_register(zb_action_handler);

    /* Configure reporting for analog input on both endpoints */
    esp_zb_zcl_reporting_info_t reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = EP_DISTANCE,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID,
        .u.send_info.min_interval = 10,
        .u.send_info.max_interval = 300,
        .u.send_info.delta.s32 = 1,
        .u.send_info.def_min_interval = 10,
        .u.send_info.def_max_interval = 300,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    esp_zb_zcl_update_reporting_info(&reporting_info);

    reporting_info.ep = EP_BATTERY;
    esp_zb_zcl_update_reporting_info(&reporting_info);

    reporting_info.ep = EP_FILL_LEVEL;
    esp_zb_zcl_update_reporting_info(&reporting_info);

    ESP_LOGI(TAG, "Zigbee device registered: EP1=distance, EP2=battery, EP3=config, EP4=fill_level, EP5=tank_height, OTA enabled");
    ESP_LOGI(TAG, "OTA: manufacturer=0x%04x, image_type=0x%04x, version=0x%08lx",
             OTA_UPGRADE_MANUFACTURER, OTA_UPGRADE_IMAGE_TYPE,
             (unsigned long)OTA_UPGRADE_FILE_VERSION);

    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));

    esp_zb_stack_main_loop();
}
