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
#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#include "soc/usb_serial_jtag_reg.h"

#include "esp_system.h"

#include "esp_err.h"
#include "esp_log.h"

#include "esp_netif.h"

#include "esp_chip_info.h"
#include "esp_flash.h"

#include "protocol_examples_common.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/err.h"
#include "lwip/sys.h"


#include "mqtt_client.h"

#include "driver/gpio.h"

#include "sdkconfig.h"

#include "ble.h"
#include "config.h"
#include "wifi.h"
#include "motor.h"
#include "mqtt.h"
#include "sensor.h"

#define APP_TAG "MOTIONAUTOMATED"

#define MQTT_QOS    1
/* User Defination */

const char *mqtt_url = "mqtt://198.211.108.247";
char pub_topic[20] = "/server/0/MA0001";
char sub_topic[20] = "/node/0/MA0001";
char id[10] = "MA0001";             // DeviceID

/* Status Flags */
int s_retry_num = 0;
int wifi_connected = 0;
int ble_config_flag = 0;
int mqtt_connected = 0;

uint8_t esp_wifi_ssid[32] = "";
uint8_t esp_wifi_pass[64] =  "";


/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */


/* Variables */
float temp = 22.5;
float low_temp = 0.0;
float high_temp = 0.0;
int status = 0;
int prev_status = 0;
uint8_t auto_mode = 0;

esp_mqtt_client_handle_t client;

/* sensor status */
uint8_t pos_start_trigger = 0;
uint8_t pos_quarter_trigger = 0;
uint8_t pos_half_trigger = 0;
uint8_t pos_three_trigger = 0;
uint8_t pos_end_trigger = 0;


/* End Defination*/

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

void app_main(void)
{
    esp_err_t ret;

    int pub_loop = 0;

    printf("MotionAutomated!\n");
    get_chip_info();

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    
    init_sensor_gpio();

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

    init_motor_gpio();
    
    clk_pwm_init();
    set_clk_duty(50);
    set_motor_dir(MOTOR_UP);
    set_motor_enb(MOTOR_ENABLE);
    fflush(stdout);
    while (1)
    {
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
                int msg_id = esp_mqtt_client_publish(client, pub_topic, pub_string, 0, MQTT_QOS, 0);
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
