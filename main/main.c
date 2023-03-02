/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/****************************************************************************
*
* This demo showcases BLE GATT server. It can send adv data, be connected by client.
* Run the gatt_client demo, the client demo will automatically connect to the gatt_server demo.
* Client demo will enable gatt_server's notify after connection. The two devices will then exchange
* data.
*
****************************************************************************/


#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include "soc/usb_serial_jtag_reg.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_bt.h"
#include "esp_chip_info.h"
#include "esp_flash.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "protocol_examples_common.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include "cJSON.h"
#include "sdkconfig.h"

#define APP_TAG "MOTIONAUTOMATED"

/* User Defination */

#define CLK_TIMER              LEDC_TIMER_0
#define CLK_MODE               LEDC_LOW_SPEED_MODE
#define CLK_OUTPUT_IO          (4) // Define the output GPIO
#define CLK_CHANNEL            LEDC_CHANNEL_0
#define CLK_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define CLK_DUTY               (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define CLK_FREQUENCY          (500) // Frequency in Hertz. Set frequency at 5 kHz

#define DIR_OUTPUT_IO           (18)
#define ENB_OUTPUT_IO           (0)

#define MOTOR_ENABLE            (0)
#define MOTOR_DISABLE           (1)

#define MOTOR_UP                (0)
#define MOTOR_DOWN              (1)

#define GPIO_INPUT_IO_0     2
#define GPIO_INPUT_IO_1     19
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))

#define EXAMPLE_ESP_MAXIMUM_RETRY  5

uint8_t EXAMPLE_ESP_WIFI_SSID[32] = "";
uint8_t EXAMPLE_ESP_WIFI_PASS[64] =  "";

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

/* Status Flags */
static int s_retry_num = 0;
static int wifi_connected = 0;
int ble_config_flag = 0;
/* Server Info */
const char *mqtt_url = "mqtt://198.211.108.247";
char pub_topic[20] = "/server/0/MA0001";
char sub_topic[20] = "/node/0/MA0001";
char id[10] = "MA0001";
/* Variables */
float temp = 22.5;
float low_temp = 0.0;
float high_temp = 0.0;
int status = 0;
int prev_status = 0;
uint8_t auto_mode = 0;
int mqtt_connected;

/* End Defination*/

///Declare the static function
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

#define GATTS_SERVICE_UUID_TEST_A   0x00FF
#define GATTS_CHAR_UUID_TEST_A      0xFF01
#define GATTS_DESCR_UUID_TEST_A     0x3333
#define GATTS_NUM_HANDLE_TEST_A     4

#define TEST_DEVICE_NAME            "MA0001" // Bluetooth Device Name

#define TEST_MANUFACTURER_DATA_LEN  17

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

#define PREPARE_BUF_MAX_SIZE 1024

static uint8_t char1_str[] = {0x11,0x22,0x33};
static esp_gatt_char_prop_t a_property = 0;

static esp_attr_value_t gatts_demo_char1_val =
{
    .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(char1_str),
    .attr_value   = char1_str,
};

static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)

#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
        0x02, 0x01, 0x06,
        0x02, 0x0a, 0xeb, 0x03, 0x03, 0xab, 0xcd
};
static uint8_t raw_scan_rsp_data[] = {
        0x0f, 0x09, 0x45, 0x53, 0x50, 0x5f, 0x47, 0x41, 0x54, 0x54, 0x53, 0x5f, 0x44,
        0x45, 0x4d, 0x4f
};
#else

static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

// The length of adv data must be less than 31 bytes
//static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
//adv data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    //.min_interval = 0x0006,
    //.max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

#endif /* CONFIG_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gatts_cb = gatts_profile_a_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    }
};

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t a_prepare_write_env;

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

/* MQTT Static Functions */

esp_mqtt_client_handle_t client;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
esp_err_t setConfigFromNVS();

void createStringFromJson(char *str)
{
    //data format: "{\"id\":\"MA0001\",\"status\":0,\"temp\":10,\"auto\":1}"
    cJSON *root;
	root = cJSON_CreateObject();
	cJSON_AddStringToObject(root, "id", id);
	cJSON_AddNumberToObject(root, "status", status);
	cJSON_AddNumberToObject(root, "temp", temp);
    cJSON_AddNumberToObject(root, "auto", auto_mode);
	char *my_json_string = cJSON_Print(root);
    strcpy(str, my_json_string);
	ESP_LOGI(APP_TAG, "my_json_string\n%s",my_json_string);
	cJSON_Delete(root);
}
void parseStringToJson(char *str)
{
    cJSON *json = cJSON_Parse(str);
    if(cJSON_GetObjectItem(json, "cmd")){
        int cmd = cJSON_GetObjectItem(json, "cmd")->valueint;
        ESP_LOGI(APP_TAG, "received command from server, value = %d", cmd);
        // run the motor
        status = cmd;
    }
    if(cJSON_GetObjectItem(json, "auto")){
        auto_mode = cJSON_GetObjectItem(json, "auto")->valueint;
        ESP_LOGI(APP_TAG, "switched mode by server, value = %d", auto_mode);
        // switch auto_mode
    }
    if(cJSON_GetObjectItem(json, "low_temp")){
        low_temp = cJSON_GetObjectItem(json, "low_temp")->valuedouble;
        ESP_LOGI(APP_TAG, "updated low temperature, value = %f", low_temp);
    }
    if(cJSON_GetObjectItem(json, "high_temp")){
        high_temp = cJSON_GetObjectItem(json, "high_temp")->valuedouble;
        ESP_LOGI(APP_TAG, "updated high temperature, value = %f", high_temp);
    }
    char pub_string[64];
    createStringFromJson(pub_string);
    int msg_id = esp_mqtt_client_publish(client, pub_topic, pub_string, 0, 0, 0);
    ESP_LOGI(APP_TAG, "sent publish successful, msg_id=%d, data=%s", msg_id, pub_string);
    cJSON_Delete(json);
}

void parseConfig(char *str)
{
    ESP_LOGI(APP_TAG, "Call parseConfig");
    cJSON *conf = cJSON_Parse(str);
    char *my_json_string = cJSON_Print(conf);
	ESP_LOGI(APP_TAG, "my_json_string\n%s",my_json_string);

    if(cJSON_GetObjectItem(conf, "ssid")){
        char *wifi_ssid = cJSON_GetObjectItem(conf, "ssid")->valuestring;
        ESP_LOGI(APP_TAG, "WiFi SSID = %s", wifi_ssid);
        memcpy(EXAMPLE_ESP_WIFI_SSID, wifi_ssid, 32);
    }
    if(cJSON_GetObjectItem(conf, "pass")){
        char *wifi_pass = cJSON_GetObjectItem(conf, "pass")->valuestring;
        ESP_LOGI(APP_TAG, "WIFI PASSWORD = %s", wifi_pass);
        memcpy(EXAMPLE_ESP_WIFI_PASS, wifi_pass, 64);
    }
    if(cJSON_GetObjectItem(conf, "auto")){
        auto_mode = cJSON_GetObjectItem(conf, "auto")->valueint;
        ESP_LOGI(APP_TAG, "AUTOMODE = %d", auto_mode);
    }
    if(cJSON_GetObjectItem(conf, "low_temp")){
        low_temp = cJSON_GetObjectItem(conf, "low_temp")->valueint;
        low_temp = low_temp / 10;
        ESP_LOGI(APP_TAG, "LOW TEMPERATURE = %d", low_temp);
    }
    if(cJSON_GetObjectItem(conf, "high_temp")){
        high_temp = cJSON_GetObjectItem(conf, "high_temp")->valueint;
        high_temp = high_temp / 10;
        ESP_LOGI(APP_TAG, "HIGH TEMPERATURE = %d", auto_mode);
    }
    setConfigFromNVS();
    cJSON_Delete(conf);
    ble_config_flag = 1;
}
static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(APP_TAG, "Last error %s: 0x%x", message, error_code);
    }
}

/* Motor Functions */
void set_motor_dir(int direction) {
    printf("Move Up\n");
    gpio_set_level(ENB_OUTPUT_IO, MOTOR_DISABLE); //stop motor

    vTaskDelay(500 / portTICK_PERIOD_MS); //Delay 500ms   
    gpio_set_level(DIR_OUTPUT_IO, direction); 
    
    gpio_set_level(ENB_OUTPUT_IO, MOTOR_ENABLE); //start again
}

void set_motor_enb(int val)
{
    if(val){
        printf("Start Motor\n");
    }
    else{
        printf("Stop Motor\n");
    }
    
    gpio_set_level(ENB_OUTPUT_IO, val);
}

void get_chip_info()
{
    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    uint32_t size_flash_chip;
    esp_flash_get_size(NULL, &size_flash_chip);
    printf("%dMB %s flash\n",  size_flash_chip / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());
    vTaskDelay(1000 / portTICK_PERIOD_MS); //Delay 1s   
}

void set_clk_duty(int duty)
{
    printf("Set Duty to %d%% \n", duty);
    int duty_val = 8191 * duty / 100;
    //set duty
    ESP_ERROR_CHECK(ledc_set_duty(CLK_MODE, CLK_CHANNEL, duty_val));
    //Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(CLK_MODE, CLK_CHANNEL));
}

static void clk_pwm_init(void)
{
    printf("Initiate the LEDC PWM Time\n");
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t clk_pwm_timer = {
        .speed_mode       = CLK_MODE,
        .timer_num        = CLK_TIMER,
        .duty_resolution  = CLK_DUTY_RES,
        .freq_hz          = CLK_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&clk_pwm_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t clk_pwm_channel = {
        .speed_mode     = CLK_MODE,
        .channel        = CLK_CHANNEL,
        .timer_sel      = CLK_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = CLK_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&clk_pwm_channel));
}

void output_io_init()
{
    printf("Initiate DIR, ENB pin to Output\n");
    gpio_reset_pin(DIR_OUTPUT_IO);
    gpio_reset_pin(ENB_OUTPUT_IO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(DIR_OUTPUT_IO, GPIO_MODE_OUTPUT);
    gpio_set_direction(ENB_OUTPUT_IO, GPIO_MODE_OUTPUT);

    gpio_set_level(DIR_OUTPUT_IO, MOTOR_UP);
    gpio_set_level(ENB_OUTPUT_IO, MOTOR_DISABLE);
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(APP_TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(APP_TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(APP_TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            //.ssid = EXAMPLE_ESP_WIFI_SSID,
            //.password = EXAMPLE_ESP_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	        .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    memcpy(wifi_config.sta.ssid, EXAMPLE_ESP_WIFI_SSID, 32);
    memcpy(wifi_config.sta.password, EXAMPLE_ESP_WIFI_PASS, 64);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(APP_TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            10000 / portTICK_PERIOD_MS);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(APP_TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
        wifi_connected = 1;

    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(APP_TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
        wifi_connected = 0;
    } else {
        ESP_LOGE(APP_TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

/* MQTT Application */
static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = mqtt_url,
        .broker.address.port = 1883,
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}
/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(APP_TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    //esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(APP_TAG, "MQTT_EVENT_CONNECTED");\
        mqtt_connected = 1;
        //msg_id = esp_mqtt_client_publish(client, "server", "data_3", 0, 1, 0);
        //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        //msg_id = esp_mqtt_client_subscribe(client, "node", 1);
        //ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(APP_TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(APP_TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(APP_TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(APP_TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(APP_TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        prev_status = status;
        parseStringToJson(event->data);
        //if(prev_status != status){
        //    set_motor_dir(status);
        //}
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(APP_TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(APP_TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(APP_TAG, "Other event id:%d", event->event_id);
        break;
    }
}

/* Bluetooth Functions */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
#ifdef CONFIG_SET_RAW_ADV_DATA
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done==0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done==0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#else
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#endif
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(APP_TAG, "Advertising start failed\n");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(APP_TAG, "Advertising stop failed\n");
        } else {
            ESP_LOGI(APP_TAG, "Stop adv successfully\n");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(APP_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.need_rsp){
        if (param->write.is_prep){
            if (prepare_write_env->prepare_buf == NULL) {
                prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE*sizeof(uint8_t));
                prepare_write_env->prepare_len = 0;
                if (prepare_write_env->prepare_buf == NULL) {
                    ESP_LOGE(APP_TAG, "Gatt_server prep no mem\n");
                    status = ESP_GATT_NO_RESOURCES;
                }
            } else {
                if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_OFFSET;
                } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_ATTR_LEN;
                }
            }

            esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK){
               ESP_LOGE(APP_TAG, "Send response error\n");
            }
            free(gatt_rsp);
            if (status != ESP_GATT_OK){
                return;
            }
            memcpy(prepare_write_env->prepare_buf + param->write.offset,
                   param->write.value,
                   param->write.len);
            prepare_write_env->prepare_len += param->write.len;

        }else{
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
    }
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC){
        esp_log_buffer_hex(APP_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
        // parse the configuration of the bluetooth
        parseConfig((char*)(prepare_write_env->prepare_buf));
    }else{
        ESP_LOGI(APP_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(APP_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_A;

        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(TEST_DEVICE_NAME);
        if (set_dev_name_ret){
            ESP_LOGE(APP_TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }
#ifdef CONFIG_SET_RAW_ADV_DATA
        esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
        if (raw_adv_ret){
            ESP_LOGE(APP_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
        }
        adv_config_done |= adv_config_flag;
        esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
        if (raw_scan_ret){
            ESP_LOGE(APP_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
        }
        adv_config_done |= scan_rsp_config_flag;
#else
        //config adv data
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret){
            ESP_LOGE(APP_TAG, "config adv data failed, error code = %x", ret);
        }
        adv_config_done |= adv_config_flag;
        //config scan response data
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret){
            ESP_LOGE(APP_TAG, "config scan response data failed, error code = %x", ret);
        }
        adv_config_done |= scan_rsp_config_flag;

#endif
        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_A);
        break;
    case ESP_GATTS_READ_EVT: {
        ESP_LOGI(APP_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 10;
        for (int i = 0; i < sizeof(id); i++){
            rsp.attr_value.value[i] = id[i];
        }
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);
        break;
    }
    case ESP_GATTS_WRITE_EVT: {
        ESP_LOGI(APP_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
        if (!param->write.is_prep){
            ESP_LOGI(APP_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
            esp_log_buffer_hex(APP_TAG, param->write.value, param->write.len);
            if (gl_profile_tab[PROFILE_A_APP_ID].descr_handle == param->write.handle && param->write.len == 2){
                uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                if (descr_value == 0x0001){
                    if (a_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY){
                        ESP_LOGI(APP_TAG, "notify enable");
                        uint8_t notify_data[15];
                        for (int i = 0; i < sizeof(notify_data); ++i)
                        {
                            notify_data[i] = i%0xff;
                        }
                        //the size of notify_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                sizeof(notify_data), notify_data, false);
                    }
                }else if (descr_value == 0x0002){
                    if (a_property & ESP_GATT_CHAR_PROP_BIT_INDICATE){
                        ESP_LOGI(APP_TAG, "indicate enable");
                        uint8_t indicate_data[15];
                        for (int i = 0; i < sizeof(indicate_data); ++i)
                        {
                            indicate_data[i] = i%0xff;
                        }
                        //the size of indicate_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                sizeof(indicate_data), indicate_data, true);
                    }
                }
                else if (descr_value == 0x0000){
                    ESP_LOGI(APP_TAG, "notify/indicate disable ");
                }else{
                    ESP_LOGE(APP_TAG, "unknown descr value");
                    esp_log_buffer_hex(APP_TAG, param->write.value, param->write.len);
                }

            }
            
        }
        example_write_event_env(gatts_if, &a_prepare_write_env, param);
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
        ESP_LOGI(APP_TAG,"ESP_GATTS_EXEC_WRITE_EVT");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        example_exec_write_event_env(&a_prepare_write_env, param);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(APP_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(APP_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_A;

        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);
        a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
                                                        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                        a_property,
                                                        &gatts_demo_char1_val, NULL);
        if (add_char_ret){
            ESP_LOGE(APP_TAG, "add char failed, error code =%x",add_char_ret);
        }
        break;
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;
    case ESP_GATTS_ADD_CHAR_EVT: {
        uint16_t length = 0;
        const uint8_t *prf_char;

        ESP_LOGI(APP_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
                param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle,  &length, &prf_char);
        if (get_attr_ret == ESP_FAIL){
            ESP_LOGE(APP_TAG, "ILLEGAL HANDLE");
        }

        ESP_LOGI(APP_TAG, "the gatts demo char length = %x\n", length);
        for(int i = 0; i < length; i++){
            ESP_LOGI(APP_TAG, "prf_char[%x] =%x\n",i,prf_char[i]);
        }
        esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].descr_uuid,
                                                                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        if (add_descr_ret){
            ESP_LOGE(APP_TAG, "add char descr failed, error code =%x", add_descr_ret);
        }
        break;
    }
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        gl_profile_tab[PROFILE_A_APP_ID].descr_handle = param->add_char_descr.attr_handle;
        ESP_LOGI(APP_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
                 param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(APP_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
                 param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT: {
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
        ESP_LOGI(APP_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
        //start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(APP_TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(APP_TAG, "ESP_GATTS_CONF_EVT, status %d attr_handle %d", param->conf.status, param->conf.handle);
        if (param->conf.status != ESP_GATT_OK){
            esp_log_buffer_hex(APP_TAG, param->conf.value, param->conf.len);
        }
        break;
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        } else {
            ESP_LOGI(APP_TAG, "Reg app failed, app_id %04x, status %d\n",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == gl_profile_tab[idx].gatts_if) {
                if (gl_profile_tab[idx].gatts_cb) {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

esp_err_t readConfigFromNVS()
{
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        printf("Done\n");

         // Read
        printf("Reading auto mode from NVS ... ");
        err = nvs_get_u8(my_handle, "auto_mode", &auto_mode);
        switch (err) {
            case ESP_OK:
                printf("Done\n");
                printf("AUTOMODE = %d\n", auto_mode);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet!\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

        printf("Reading low temperature from NVS ... ");
        err = nvs_get_i16(my_handle, "low_temp", &low_temp);
        switch (err) {
            case ESP_OK:
                printf("Done\n");
                low_temp = low_temp / 10;
                printf("LOW TEMPERATURE = %f\n", low_temp);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet!\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

        printf("Reading high temperature from NVS ... ");
        err = nvs_get_i16(my_handle, "high_temp", &high_temp);
        switch (err) {
            case ESP_OK:
                printf("Done\n");
                high_temp = high_temp / 10;
                printf("HIGH TEMPERATURE = %f\n", high_temp);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet!\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

        printf("Reading length of wifi ssid from NVS ... ");
        size_t ssid_length = sizeof(EXAMPLE_ESP_WIFI_SSID);
        err = nvs_get_u8(my_handle, "ssid_length", &ssid_length);
        switch (err) {
            case ESP_OK:
                printf("Done\n");
                printf("SSID Length = %d\n", ssid_length);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet!\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

        printf("Reading wifi ssid from NVS ... ");
        err = nvs_get_str(my_handle, "wifi_ssid", (char*)EXAMPLE_ESP_WIFI_SSID, &ssid_length);
        switch (err) {
            case ESP_OK:
                printf("Done\n");
                printf("WIFI SSID = %s\n", EXAMPLE_ESP_WIFI_SSID);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet!\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

        printf("Reading length of wifi pass from NVS ... ");
        size_t pass_length = sizeof(EXAMPLE_ESP_WIFI_PASS);
        err = nvs_get_u8(my_handle, "pass_length", &pass_length);
        switch (err) {
            case ESP_OK:
                printf("Done\n");
                printf("WiFi PASS Length = %d\n", pass_length);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet!\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

        printf("Reading wifi password from NVS ... ");
        err = nvs_get_str(my_handle, "wifi_pass", (char*)EXAMPLE_ESP_WIFI_PASS, &pass_length);
        switch (err) {
            case ESP_OK:
                printf("Done\n");
                printf("WIFI PASSWORD = %s\n", EXAMPLE_ESP_WIFI_PASS);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet!\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }
       
         // Close
        nvs_close(my_handle);
    }
    return err;

} 

esp_err_t setConfigFromNVS()
{
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        printf("Done\n");

         // Read
        printf("Set auto mode from NVS ... ");
        err = nvs_set_u8(my_handle, "auto_mode", auto_mode);
        if (err != ESP_OK) return err;
        else{
            printf("Done\n");
            printf("AUTOMODE = %d\n", auto_mode);
        }

        printf("Set low temperature from NVS ... ");
        uint16_t tmp_low_temp = low_temp * 10;
        err = nvs_set_i16(my_handle, "low_temp", tmp_low_temp);
        if (err != ESP_OK) return err;
        else {
            printf("Done\n");
                printf("LOW TEMPERATURE = %f\n", low_temp);
        }

        printf("Set high temperature from NVS ... ");
        uint16_t tmp_high_temp = high_temp * 10;
        err = nvs_set_i16(my_handle, "high_temp", tmp_high_temp);
        if (err != ESP_OK) return err;
        else{
            printf("Done\n");
            printf("HIGH TEMPERATURE = %f\n", high_temp);
        }

        printf("Set length of wifi ssid from NVS ... ");
        uint8_t ssid_length = sizeof(EXAMPLE_ESP_WIFI_SSID);
        err = nvs_set_u8(my_handle, "ssid_length", ssid_length);
        if (err != ESP_OK) return err;
        else{
            printf("Done\n");
        }

        printf("Set wifi ssid from NVS ... ");
        err = nvs_set_str(my_handle, "wifi_ssid", (char*)EXAMPLE_ESP_WIFI_SSID);
        if (err != ESP_OK) return err;
        else {
            printf("Done\n");
            printf("WIFI SSID = %s\n", EXAMPLE_ESP_WIFI_SSID);
        }

        printf("Set length of wifi pass from NVS ... ");
        uint8_t pass_length = sizeof(EXAMPLE_ESP_WIFI_PASS);
        err = nvs_set_u8(my_handle, "pass_length", pass_length);
        if (err != ESP_OK) return err;
        else{
            printf("Done\n");
        }

        printf("Set wifi password from NVS ... ");
        err = nvs_set_str(my_handle, "wifi_pass", (char*)EXAMPLE_ESP_WIFI_PASS);
        if (err != ESP_OK) return err;
        else {
            printf("Done\n");
            printf("WIFI PASSWORD = %s\n", EXAMPLE_ESP_WIFI_PASS);
        }

        err = nvs_commit(my_handle);
        if (err != ESP_OK) {
            printf("Commiting to NVS is failed\n");  
            return err;
         }// Close
        printf("Commiting Done\n");  
        nvs_close(my_handle);
    }
    return err;

} 

void app_main(void)
{
    esp_err_t ret;

    int pub_loop = 0;
    int gpio_input_status0 = 0;
    int gpio_input_status1 = 0;

    printf("MotionAutomated!\n");
    get_chip_info();

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    /* Button to switch the configuration mode */

    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //bit mask of the pins, use GPIO2/19 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;

    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    readConfigFromNVS();

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(APP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(APP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(APP_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(APP_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(APP_TAG, "gatts register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(APP_TAG, "gap register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
    if (ret){
        ESP_LOGE(APP_TAG, "gatts app register error, error code = %x", ret);
        return;
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(APP_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
    ESP_LOGI(APP_TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
    if(wifi_connected){
        ESP_LOGI(APP_TAG, "Initiate MQTT Module");
        mqtt_app_start();
    }  
    clk_pwm_init();
    set_clk_duty(50);
    set_motor_enb(MOTOR_DISABLE);
    //set_motor_dir(MOTOR_UP);
    fflush(stdout);
    while (1)
    {
        gpio_input_status0 = gpio_get_level(GPIO_INPUT_IO_0);
        gpio_input_status1 = gpio_get_level(GPIO_INPUT_IO_1);
        //ESP_LOGI(APP_TAG, "GPIO Input =%d, data=%d", GPIO_INPUT_IO_0 , gpio_input_status0);
        //ESP_LOGI(APP_TAG, "GPIO Input =%d, data=%d", GPIO_INPUT_IO_1 , gpio_input_status1);
        if(ble_config_flag == 1){
            vTaskDelay(3000 / portTICK_PERIOD_MS);
            esp_restart();
        }  
        if(mqtt_connected){
            // Publish the Node status
            if(pub_loop == 5){
                char pub_string[64];
                createStringFromJson(pub_string);
                int msg_id = esp_mqtt_client_publish(client, pub_topic, pub_string, 0, 0, 0);
                ESP_LOGI(APP_TAG, "sent publish successful, msg_id=%d, data=%s", msg_id, pub_string);
                pub_loop = 0;
            }
            //Subscribe the Server command
            esp_mqtt_client_subscribe(client, sub_topic, 0);

            pub_loop++;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
