#ifndef MQTT_H_
#define MQTT_H_

#include "mqtt_client.h"
#include "esp_log.h"
#include "esp_err.h"

#include "config.h"

#define MQTT_QOS    1

extern esp_mqtt_client_handle_t client;

extern const char *mqtt_url;
extern char pub_topic[20];
extern char sub_topic[20];
extern int mqtt_connected;

void mqtt_app_start(void);
void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
void publish_status(void);

#endif