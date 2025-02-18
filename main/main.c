/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee HA_on_off_light Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_zigbee_core.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "driver/gpio.h"
#include "main.h"
#include "clock.h"
#include "ld2420_comm.h"
#include "triac_dimmer.h"

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile light (End Device) source code.
#endif

static const char *TAG = "APP_MAIN";



enum _operation_modes {
    OPMODE_AUTO = 1,
    OPMODE_ON = 2,
    OPMODE_OFF = 3,
};

static uint16_t operation_mode = OPMODE_AUTO;
static bool enable_dimmed_mode = false;
static int prev_motion_detected = 0;
static int light_state = 1, prev_light_state = 0;
static int32_t light_delay = 2; // seconds, delay after motion detection
static int32_t light_timeout = 10; // seconds, time to keep the light on after motion has stopped
static int32_t light_dim_level = DIM_LEVEL_MAX/2;

static uint64_t light_time_on = 0;
static uint64_t no_motion_time = 0;
static gptimer_handle_t timer = 0;
static const uint32_t timer_resolution = 10 * 1000;

static const unsigned indication_led_pin = 13;

TaskHandle_t dimmerTaskHandle = NULL;

/********************* Define functions **************************/
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

static const int ZB_INIT_FAIL_COUNT_TO_REBOOT = 60;
static const int ZB_ZDO_FAIL_COUNT_TO_REBOOT = 10;
static int zb_fail_count = 0;


static struct {
    char     description[16];
    float    min_present;
    float    max_present;
    float    present_value;
    float    resolution;
    uint16_t status_flags;
    uint16_t units;
    uint32_t app_type;
    volatile int32_t *value_ptr;
    char     nvs_key[8];
} analog_attr_values[] = {
    {
    .description = "\x07" "Timeout",
    .min_present = 1,
    .max_present = 3600,
    .present_value = 20,
    .resolution = 1,
    .status_flags = 0,
    .units = 73, /* 73 = seconds */
    .app_type = 14 << 16, /* 14 = timer */
//    .value_ptr = &motionTimeout,
    .value_ptr = &light_timeout,
    .nvs_key = "AO_00",
    },

    {
    .description = "\x05" "Delay",
    .min_present = 0,
    .max_present = 20,
    .present_value = 2,
    .resolution = 1,
    .status_flags = 0,
    .units = 73, /* 73 = seconds */
    .app_type = 14 << 16, /* 14 = timer */
    .value_ptr = &light_delay,
    .nvs_key = "AO_01",
    },

    {
    .description = "\x0e" "Distance range",
    .min_present = 1,
    .max_present = 1000,
    .present_value = 200,
    .resolution = 1,
    .status_flags = 0,
    .units = 118, /* 118 = cm */
    .app_type = 2 << 16, /* 2 = Gauge */
    .value_ptr = &motionThreshold,
    .nvs_key = "AO_02",
    },

    {
    .description = "\x09" "Dim level",
    .min_present = 0,
    .max_present = 100,
    .present_value = 50,
    .resolution = 1,
    .status_flags = 0,
    .units = 98, /* 98 = % */
    .app_type = 15 << 16, /* 15 = palette */
    .value_ptr = &light_dim_level,
    .nvs_key = "AO_03",
    },

    {
    .description = "\x0b" "Phase corr.",
    .min_present = -10000,
    .max_present = 10000,
    .present_value = 0,
    .resolution = 1,
    .status_flags = 0,
    .units = 194, /* 194 = microseconds */
    .app_type = 14 << 16, /* 15 = timer */
    .value_ptr = &phaseCorrection_us,
    .nvs_key = "AO_04",
    },

    {
    .description = "\x0a" "Transition",
    .min_present = 0,
    .max_present = 15,
    .present_value = 0,
    .resolution = 1,
    .status_flags = 0,
    .units = 95,
    .app_type = 14 << 16, /* 15 = timer */
    .value_ptr = &transitionTime,
    .nvs_key = "AO_05",
    },


};



static struct {
    char     *states_text;
    char     description[16];
    uint16_t *present_value_ptr;
    uint16_t status_flags;
    uint16_t number_of_states;
    char     nvs_key[8];
} multistate_attr_values[] = {
    {

/* Binary format for the ZCL string array: 1. type; 2. count; 3. array of strings.
*/
    .states_text = "\x42"       /*ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING = 0x42 */
                    "\x04\x00"  /* Element count */
                    "\x04" "Auto"
                    "\x02" "On"
                    "\x03" "Off",
    .description = "\x0e" "Operation mode",
    .present_value_ptr = &operation_mode,
    .status_flags = 0,
    .number_of_states = 3,
    .nvs_key = "MV_00",
    }
};

char light_on_binary_desc[] = "\x0b" "Light is on";
char motion_binary_desc[] = "\x0f" "Motion detected";

const char ZB_STORAGE_NAMESPACE[] = "zb_storage";

static void init_gpio(void)
{
    const gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << indication_led_pin),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };

    gpio_config(&io_conf);
    gpio_set_level(indication_led_pin, 1);
}

/* Signal the dimmer task to disable its sync source and stop the timer
    to avoid light flash when the source pin is disabled at restart. */
static void stop_dimmer_and_restart(void)
{
    stopDimmer = 1;
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
}


void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    uint8_t min_lqi;

    switch (sig_type) {
    case ESP_ZB_NLME_STATUS_INDICATION:
        ESP_LOGI(TAG, "%s, status: 0x%x\n", esp_zb_zdo_signal_to_string(sig_type), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));
        break;
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialized");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        min_lqi = esp_zb_secur_network_min_join_lqi_get();
        ESP_LOGI(TAG, "Min LQI = %u", (unsigned)min_lqi);
        esp_zb_secur_network_min_join_lqi_set(0);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device rebooted");
            }

            zb_fail_count = 0;
        } else {
            /* commissioning failed */

            if (ZB_INIT_FAIL_COUNT_TO_REBOOT <= zb_fail_count) {
                ESP_LOGI(TAG, "ZB init has failed too many times. Restarting.");
                stop_dimmer_and_restart();
                break;
            }

            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
            zb_fail_count += 1;
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        ESP_LOGI(TAG, "Leave and network steering initiated.");
        esp_zb_factory_reset();
        esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        break;
    case ESP_ZB_ZDO_DEVICE_UNAVAILABLE:
        if (ZB_ZDO_FAIL_COUNT_TO_REBOOT <= zb_fail_count) {
            ESP_LOGI(TAG, "ZDO device unavailable. Restarting.");
            stop_dimmer_and_restart();
            break;
        }
        zb_fail_count += 1;
        __attribute__ ((fallthrough));
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static struct {
    char     description[16];
    float    min_present;
    float    max_present;
    float    present_value;
    float    resolution;
    uint16_t status_flags;
    uint16_t units;
    uint32_t app_type;
} distance_sensor_attr_values = {
    .description = DISTANCE_SENSOR_DESC,
    .min_present = 0,
    .max_present = 1000,
    .present_value = 0,
    .resolution = 1,
    .status_flags = 0,
    .units = 118, /* 118 = cm */
    .app_type = 2 << 16, /* 2 = Gauge icon */
};

static struct {
    int id;
    void *ptr;
    int access;
} distance_sensor_attr_data[] = {
    { ESP_ZB_ZCL_ATTR_ANALOG_INPUT_DESCRIPTION_ID      , &distance_sensor_attr_values.description,   .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY },
    { ESP_ZB_ZCL_ATTR_ANALOG_INPUT_MAX_PRESENT_VALUE_ID, &distance_sensor_attr_values.max_present,   .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY },
    { ESP_ZB_ZCL_ATTR_ANALOG_INPUT_MIN_PRESENT_VALUE_ID, &distance_sensor_attr_values.min_present,   .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY },
    { ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID    , &distance_sensor_attr_values.present_value, .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING },
    { ESP_ZB_ZCL_ATTR_ANALOG_INPUT_RESOLUTION_ID       , &distance_sensor_attr_values.resolution,    .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY },
    { ESP_ZB_ZCL_ATTR_ANALOG_INPUT_STATUS_FLAGS_ID     , &distance_sensor_attr_values.status_flags,  .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE },
    { ESP_ZB_ZCL_ATTR_ANALOG_INPUT_ENGINEERING_UNITS_ID, &distance_sensor_attr_values.units,         .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY },
    { ESP_ZB_ZCL_ATTR_ANALOG_INPUT_APPLICATION_TYPE_ID , &distance_sensor_attr_values.app_type,      .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY }
};

static  esp_zb_zcl_reporting_info_t reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = 0, //HA_ANALOG_INPUT_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 0,
        .u.send_info.max_interval = 10,
        .u.send_info.def_min_interval = 0,
        .u.send_info.def_max_interval = 10,
        .u.send_info.delta.u16 = 1,
        .attr_id = ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };


#define MULTISTATE_ATTR_FIELD_OFFSET(field) (offsetof(typeof(multistate_attr_values[0]), field))

static struct {
    int id;
    int offs;
    int access;
} multistate_attr_offset[] = {
    { ESP_ZB_ZCL_ATTR_MULTI_VALUE_DESCRIPTION_ID      , MULTISTATE_ATTR_FIELD_OFFSET(description),   .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY },
//    { ESP_ZB_ZCL_ATTR_MULTI_VALUE_PRESENT_VALUE_ID    , MULTISTATE_ATTR_FIELD_OFFSET(present_value), .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING },
    { ESP_ZB_ZCL_ATTR_MULTI_VALUE_STATUS_FLAGS_ID     , MULTISTATE_ATTR_FIELD_OFFSET(status_flags),  .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE },
/* States text attribute is not supported... */
};

static esp_err_t nvs_save_int_attribute(int32_t value, const char *key)
{
    nvs_handle_t handle = 0;
    esp_err_t err = ESP_OK;

//    ESP_LOGI(TAG, "Saving attribute: %s = %ld", key, value);
    ESP_RETURN_ON_ERROR(nvs_open(ZB_STORAGE_NAMESPACE, NVS_READWRITE, &handle), TAG, "Error opening NVS handle!");

    err = nvs_set_i32(handle, key, value);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write to NVS (%s)!", esp_err_to_name(err));

    } else {

        err = nvs_commit(handle);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to commit to NVS (%s)!", esp_err_to_name(err));
        }
    }

    nvs_close(handle);
    return err;
}

static esp_zb_attribute_list_t *create_multistate_cluster(nvs_handle_t handle)
{
    esp_zb_attribute_list_t *esp_zb_multi_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_MULTI_VALUE);

    if (esp_zb_multi_cluster != NULL) {
/* Assemble the cluster from the attributes one by one, because the helper function (esp_zb_multistate_value_cluster_create)
    does not set ACCESS_REPORTING flag on "present_value" attribute.
*/
        int i;
        for (i = 0; i < lengthof(multistate_attr_offset); i++) {
	    esp_zb_multistate_value_cluster_add_attr(esp_zb_multi_cluster, multistate_attr_offset[i].id,
	        ((void*)&multistate_attr_values[0]) + multistate_attr_offset[i].offs );
        }

/*  Attempt to find a saved attribute value */
        int32_t saved_value = 0;
        if (handle && ESP_OK == nvs_get_i32(handle, multistate_attr_values[0].nvs_key, &saved_value)) {
            ESP_LOGI(TAG, "Loaded attribute: %s = %ld", multistate_attr_values[0].nvs_key, saved_value);

/* Do not start up in disabled state */
            if (saved_value == OPMODE_OFF || saved_value > multistate_attr_values[0].number_of_states)
                saved_value = OPMODE_AUTO;

            *multistate_attr_values[0].present_value_ptr = saved_value;
        }

        esp_zb_cluster_add_attr(esp_zb_multi_cluster,
	    ESP_ZB_ZCL_CLUSTER_ID_MULTI_VALUE,
	    ESP_ZB_ZCL_ATTR_MULTI_VALUE_PRESENT_VALUE_ID,
	    ESP_ZB_ZCL_ATTR_TYPE_U16,
	    ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
	    multistate_attr_values[0].present_value_ptr);

    }

    ESP_LOGI(TAG, "Multistate cluster created: %d", (int)(esp_zb_multi_cluster!=NULL));
    return esp_zb_multi_cluster;
}

#define ANALOG_ATTR_FIELD_OFFSET(field) (offsetof(typeof(analog_attr_values[0]), field))

static struct {
    int id;
    int offs;
    int access;
} analog_attr_offset[] = {
    { ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_DESCRIPTION_ID      , ANALOG_ATTR_FIELD_OFFSET(description),   .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY },
    { ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_MAX_PRESENT_VALUE_ID, ANALOG_ATTR_FIELD_OFFSET(max_present),   .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY },
    { ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_MIN_PRESENT_VALUE_ID, ANALOG_ATTR_FIELD_OFFSET(min_present),   .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY },
    { ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID    , ANALOG_ATTR_FIELD_OFFSET(present_value), .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING },
    { ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_RESOLUTION_ID       , ANALOG_ATTR_FIELD_OFFSET(resolution),    .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY },
    { ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_STATUS_FLAGS_ID     , ANALOG_ATTR_FIELD_OFFSET(status_flags),  .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE },
    { ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_ENGINEERING_UNITS_ID, ANALOG_ATTR_FIELD_OFFSET(units),         .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY },
    { ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_APPLICATION_TYPE_ID , ANALOG_ATTR_FIELD_OFFSET(app_type),      .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY }
};

/* Setup AnalogOutput clusters and add them to the endpoints starting from the provided index. */
static esp_zb_attribute_list_t * create_analog_cluster(unsigned idx, nvs_handle_t handle)
{
    esp_zb_attribute_list_t *esp_zb_analog_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT);
    int nattrs = 0;
    if (esp_zb_analog_cluster != NULL) {

/*  Attempt to find a saved attribute value */
        int32_t saved_value = 0;
        if (handle && ESP_OK == nvs_get_i32(handle, analog_attr_values[idx].nvs_key, &saved_value)) {
            *analog_attr_values[idx].value_ptr = saved_value;
            ESP_LOGI(TAG, "Loaded attribute: %s = %ld", analog_attr_values[idx].nvs_key, saved_value);
        }
        analog_attr_values[idx].present_value = *analog_attr_values[idx].value_ptr;

        int i;
        for (i = 0; i < lengthof(analog_attr_offset); i++) {
	    esp_err_t err = esp_zb_analog_output_cluster_add_attr(esp_zb_analog_cluster, analog_attr_offset[i].id,
	        ((void*)&analog_attr_values[idx]) + analog_attr_offset[i].offs );
	    if (ESP_OK == err)
	        nattrs += 1;
        }
    }

    ESP_LOGI(TAG, "Analog cluster %u created. Attrs: %d", idx, nattrs);
    return esp_zb_analog_cluster;
}

/* Setup AnalogInput cluster */
static esp_zb_attribute_list_t * create_analog_input_cluster(void)
{
    esp_zb_attribute_list_t *esp_zb_analog_input_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT);
    int i;
    for (i = 0; i < sizeof(distance_sensor_attr_data) / sizeof(distance_sensor_attr_data[0]); i++) {
	esp_zb_analog_input_cluster_add_attr(esp_zb_analog_input_cluster, distance_sensor_attr_data[i].id, distance_sensor_attr_data[i].ptr);
    }

    return esp_zb_analog_input_cluster;
}

/* Setup OnOff switch cluster */
static esp_zb_attribute_list_t * create_on_off_cluster(nvs_handle_t handle)
{

    esp_zb_on_off_cluster_cfg_t on_off_cfg = { .on_off = false };

    esp_zb_attribute_list_t *on_off_cluster = esp_zb_on_off_cluster_create(&on_off_cfg);

    return on_off_cluster;
}


static esp_zb_attribute_list_t * create_binary_input_cluster(char *desc, int value)
{
    esp_zb_binary_input_cluster_cfg_t cfg = { false, 0 };
    esp_zb_attribute_list_t * binary_input_cluster = esp_zb_binary_input_cluster_create(&cfg);
    if (desc) {
        ESP_ERROR_CHECK(esp_zb_binary_input_cluster_add_attr(
            binary_input_cluster,
            ESP_ZB_ZCL_ATTR_BINARY_INPUT_DESCRIPTION_ID,
            desc
        ));
    }
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(binary_input_cluster,
	    ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
	    ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,
	    ESP_ZB_ZCL_ATTR_TYPE_BOOL,
	    ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
	    &value));

    return binary_input_cluster;
}

/* Setup Basic cluster */
static esp_zb_attribute_list_t * create_basic_cluster(char *manufacturer)
{
    uint8_t zero = 0;
    uint8_t version = 3;

    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);

    if (esp_zb_basic_cluster != NULL) {
        esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, manufacturer);
        esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER);
        esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &version);
        esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &zero);
    }
    return esp_zb_basic_cluster;
}

static void setup_reporting(void)
{

/* Config the reporting info  */
    esp_zb_zcl_reporting_info_t multistate_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_MULTISTATE_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_MULTI_VALUE,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 2, // Minimum interval of 30 seconds
        .u.send_info.max_interval = 600, // Maximum interval of 10 minutes
        .u.send_info.def_min_interval = 2, // Default minimum interval of 30 seconds
        .u.send_info.def_max_interval = 600, // Default maximum interval of 10 minutes
        .u.send_info.delta.u16 = 1, // Report on every state change
        .attr_id = ESP_ZB_ZCL_ATTR_MULTI_VALUE_PRESENT_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC, //0xFFFF,
    };

    // Attempt to update the reporting info
    esp_err_t err = esp_zb_zcl_update_reporting_info(&multistate_reporting_info);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update reporting info: %s", esp_err_to_name(err));
        // Handle error appropriately
    } else {
        ESP_LOGI(TAG, "Successfully updated reporting info");
    }

 /* Config the reporting info  */
    if (reporting_info.ep > 0)
        ESP_ERROR_CHECK(esp_zb_zcl_update_reporting_info(&reporting_info));


    esp_zb_zcl_reporting_info_t binary_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_LIGHT_ON_BINARY_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1, // Minimum interval of 30 seconds
        .u.send_info.max_interval = 60, // Maximum interval of 10 minutes
        .u.send_info.def_min_interval = 1, // Default minimum interval of 30 seconds
        .u.send_info.def_max_interval = 60, // Default maximum interval of 10 minutes
        .u.send_info.delta.u16 = 1, // Report on every state change
        .attr_id = ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC, //0xFFFF,
    };

    ESP_ERROR_CHECK(esp_zb_zcl_update_reporting_info(&binary_reporting_info));

    binary_reporting_info.ep = HA_MOTION_BINARY_ENDPOINT;
    ESP_ERROR_CHECK(esp_zb_zcl_update_reporting_info(&binary_reporting_info));
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);

/* Process analog output clusters */
    if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT
        && message->attribute.id == ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID
        && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_SINGLE)
    {
        uint8_t endpoint = message->info.dst_endpoint;
        if (endpoint >= HA_FIRST_ENDPOINT && endpoint < HA_FIRST_ENDPOINT + lengthof(analog_attr_values)) {
            if (message->attribute.data.value) {
                float value = *(float *)message->attribute.data.value;
                int32_t intvalue = (int32_t)((value / analog_attr_values[endpoint - HA_FIRST_ENDPOINT].resolution) + 0.5);
                * analog_attr_values[endpoint - HA_FIRST_ENDPOINT].value_ptr = intvalue;
                nvs_save_int_attribute(intvalue, analog_attr_values[endpoint - HA_FIRST_ENDPOINT].nvs_key);
            }
        }
    }

/* Process multistate cluster */
    if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_MULTI_VALUE
        && message->attribute.id == ESP_ZB_ZCL_ATTR_MULTI_VALUE_PRESENT_VALUE_ID
        && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16)
    {
        uint8_t endpoint = message->info.dst_endpoint;
        if (endpoint == HA_MULTISTATE_ENDPOINT) {
            if (message->attribute.data.value) {
                uint16_t value = *(uint16_t *)message->attribute.data.value;
                if (value > 0 && value <= multistate_attr_values[0].number_of_states) { /* 1-based states */
                    *multistate_attr_values[0].present_value_ptr = value;
                    nvs_save_int_attribute(value, multistate_attr_values[0].nvs_key);
                }
            }
        }
    }


    if (message->info.dst_endpoint == HA_ON_OFF_DIMMER_ENDPOINT) {
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
                enable_dimmed_mode = message->attribute.data.value ? *(bool *)message->attribute.data.value : enable_dimmed_mode;
                ESP_LOGI(TAG, "Dimmer set to %s", enable_dimmed_mode ? "On" : "Off");
            }
        }
    }
    return ret;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

static void setup_endpoints(void)
{
    nvs_handle_t handle = 0;
    esp_err_t err;

    err = nvs_open(ZB_STORAGE_NAMESPACE, NVS_READONLY, &handle);
    if (ESP_OK != err) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        handle = 0;
    }

    esp_zb_ep_list_t *endpoint_list = esp_zb_ep_list_create();

    int a_idx, ep_idx;
    for (a_idx = 0, ep_idx = HA_FIRST_ENDPOINT; a_idx < lengthof(analog_attr_values); a_idx++, ep_idx++ ) {

        esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

        ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(
            cluster_list,
            create_basic_cluster(MANUFACTURER_NAME),
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
        ));

        if (ep_idx == HA_FIRST_ENDPOINT) {

            ESP_ERROR_CHECK(esp_zb_cluster_list_add_analog_input_cluster(
                cluster_list,
                create_analog_input_cluster(),
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
            ));

            reporting_info.ep = ep_idx;

            ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(
                cluster_list,
                create_on_off_cluster(handle),
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
            ));
        }

        if (ep_idx == HA_MULTISTATE_ENDPOINT) {
            ESP_ERROR_CHECK(esp_zb_cluster_list_add_multistate_value_cluster(
                cluster_list,
                create_multistate_cluster(handle),
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
            ));
        }

        ESP_ERROR_CHECK(esp_zb_cluster_list_add_analog_output_cluster(
            cluster_list,
            create_analog_cluster(a_idx, handle),
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
        ));


        if (ep_idx == HA_LIGHT_ON_BINARY_ENDPOINT) {
            ESP_ERROR_CHECK(esp_zb_cluster_list_add_binary_input_cluster(
                cluster_list,
                create_binary_input_cluster(light_on_binary_desc, light_state),
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
            ));
        }

        if (ep_idx == HA_MOTION_BINARY_ENDPOINT) {
            ESP_ERROR_CHECK(esp_zb_cluster_list_add_binary_input_cluster(
                cluster_list,
                create_binary_input_cluster(motion_binary_desc, motionDetected),
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
            ));
        }

/* Finally setup a new endpoint */
        esp_zb_endpoint_config_t ep_config = {
            .endpoint = ep_idx,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
            .app_device_version = 1,
        };


        ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(
            endpoint_list,
            cluster_list,
            ep_config
        ));

    }

    if (handle)
        nvs_close(handle);

/* Register endpoints */

    ESP_ERROR_CHECK(esp_zb_device_register(endpoint_list));
    ESP_LOGW(TAG, "Device registered");

    setup_reporting();

}

static void esp_zb_task(void *pvParameters)
{
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);

#if CONFIG_ESP_ZB_TRACE_ENABLE
   esp_zb_set_trace_level_mask(ESP_ZB_TRACE_LEVEL_DEBUG, ESP_ZB_TRACE_SUBSYSTEM_ZCL | ESP_ZB_TRACE_SUBSYSTEM_NWK | ESP_ZB_TRACE_SUBSYSTEM_TRANSPORT | ESP_ZB_TRACE_SUBSYSTEM_ZDO);
#endif

    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    setup_endpoints();

    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));

/* Trigger report */
    int value = 1;
    esp_zb_zcl_set_attribute_val(HA_FIRST_ENDPOINT+1, ESP_ZB_ZCL_CLUSTER_ID_MULTI_VALUE, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_MULTI_VALUE_PRESENT_VALUE_ID, &value, 0);

    esp_zb_stack_main_loop();
}

static void check_light_state(void)
{
    int prev = dimLevel;
    static uint64_t dbg_timer = 0;


    uint64_t clock_now = gpclock(timer);

    if (!motionDetected)
        no_motion_time = clock_now;

    switch (operation_mode) {

/* Turn light off now */
        case OPMODE_OFF:
            light_state = 0;
            break;

/* Turn light on if motion detected */
        case OPMODE_AUTO:

            if ((clock_now - no_motion_time) > ((uint64_t)light_delay * timer_resolution)) {
                light_state = 1;
            } else {
                if (clock_now - light_time_on >= (uint64_t)light_timeout * timer_resolution)
                    light_state = 0;
            }

            if (light_state && motionDetected) {
                light_time_on = clock_now;
            }
            break;

/* Turn light on now */
        case OPMODE_ON:
            light_state = 1;
            break;

        default:
            break;
    }

    if (0 != light_state)
    {
        dimLevel = enable_dimmed_mode ? light_dim_level : DIM_LEVEL_MAX;
    } else {
        dimLevel = 0;
    }


    if (prev != dimLevel) {
        ESP_LOGI(TAG, "Light is set to %s", dimLevel ? "On" : "Off");
        light_driver_set_power((dimLevel * 255 + DIM_LEVEL_MAX/2) / DIM_LEVEL_MAX);
    }

    if (clock_now > dbg_timer) {
        dbg_timer += timer_resolution;
        ESP_LOGI(TAG, "Mode %d, motion: %d, light: %d, dim: %ld, nmT: %lu, lT: %lu", (int)operation_mode, motionDetected, light_state, dimLevel,
            (uint32_t)((clock_now-no_motion_time)/timer_resolution), (uint32_t)((clock_now-light_time_on)/timer_resolution));
    }
}

static void light_control_task(void *pvParameters)
{
    static const char *TASK_TAG = "CTRL_TASK";
    uint32_t prev_dist_sensor = 0;
    while (1) {

        check_light_state();
        int have_lock = 0;

        if (prev_light_state != !!dimLevel) {
            if (have_lock || esp_zb_lock_acquire(portMAX_DELAY) ) {
                have_lock = 1;
                prev_light_state = !!dimLevel;
                esp_zb_zcl_set_attribute_val(HA_LIGHT_ON_BINARY_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
                    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID, &prev_light_state, false);
            }
        }

        if (prev_motion_detected != motionDetected) {
            if (have_lock || esp_zb_lock_acquire(portMAX_DELAY) ) {
                have_lock = 1;
                prev_motion_detected = motionDetected;
                esp_zb_zcl_set_attribute_val(HA_MOTION_BINARY_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
                    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID, &prev_motion_detected, false);
            }
        }


	if (prev_dist_sensor != ld2420_Distance) {
	    if (have_lock || esp_zb_lock_acquire(portMAX_DELAY) ) {
	        have_lock = 1;

		prev_dist_sensor = ld2420_Distance;
		float new_value = prev_dist_sensor;
		esp_zb_zcl_status_t res = esp_zb_zcl_set_attribute_val(HA_ANALOG_INPUT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
		    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &new_value, false);

		ESP_LOGI(TASK_TAG, "R: '%lu' %d", prev_dist_sensor, (int)res);
	    }
	}

        if (have_lock) {
	    esp_zb_lock_release();
	    have_lock = 0;
        }

	vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    init_gpio();

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    light_driver_init(LIGHT_DEFAULT_OFF);
    init_timer(&timer, timer_resolution);

    xTaskCreate(esp_zb_task, "Zigbee_main", 1024*8, NULL, 5, NULL);
    xTaskCreate(ld2420_task, "uart_rx_task", 1024 * 4, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(triac_dimmer_task, "dimmer_task", 1024 * 4, NULL, configMAX_PRIORITIES - 1, &dimmerTaskHandle);

    gpio_set_level(indication_led_pin, 0);

    if (ESP_RST_SW == esp_reset_reason()) /* Do not turn the light on after soft reset */
        light_state = 0;

    light_control_task(NULL);
}
