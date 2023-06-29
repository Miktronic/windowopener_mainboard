#ifndef WIFI_H_
#define WIFI_H_

#include <string.h>
#include <stdint.h>

#include "wifi.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "freertos/event_groups.h"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define EXAMPLE_ESP_MAXIMUM_RETRY  5

extern uint8_t esp_wifi_ssid[32];
extern uint8_t esp_wifi_pass[64];
extern int s_retry_num;
extern int wifi_connected;

/* FreeRTOS event group to signal when we are connected*/

void wifi_init_sta(void);

#endif