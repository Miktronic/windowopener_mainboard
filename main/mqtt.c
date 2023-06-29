#include "mqtt.h"

#define APP_TAG     "MA_MQTT_STACK"

/* MQTT Application */

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(APP_TAG, "Last error %s: 0x%x", message, error_code);
    }
}

/* Initiate the MQTT client */
void mqtt_app_start(void)
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
void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
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
        printf(" >>> TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf(" >>> DATA=%.*s\r\n", event->data_len, event->data);
        prev_status = status;
        parseString(event->data);
        publish_status();
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

/* Publish the Status of the Device to the MQTT server */
void publish_status(void)
{
    char pub_string[64];
    createStringFromJson(pub_string);
    int msg_id = esp_mqtt_client_publish(client, pub_topic, pub_string, 0, 0, 0);
    ESP_LOGI(APP_TAG, "sent publish successful, msg_id=%d, data=%s", msg_id, pub_string);
}

