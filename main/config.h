#ifndef CONFIG_H_
#define CONFIG_H_

#include <string.h>
#include <stdint.h>

#include "nvs_flash.h"
#include "cJSON.h"
#include "esp_log.h"
#include "esp_err.h"

extern uint8_t esp_wifi_ssid[32];
extern uint8_t esp_wifi_pass[64];

extern uint8_t auto_mode;
extern float low_temp;
extern float high_temp;
extern int prev_status;
extern int status;
extern float temp;
extern char id[10];

esp_err_t readConfigFromNVS();
esp_err_t setConfigFromNVS();
void createStringFromJson(char *str);
void parseString(char *str);
void publish_status();
int parseConfig(char *str);

#endif