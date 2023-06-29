#include "config.h"

#define APP_TAG "MA_DATA_INFO"

/* Read the setting variables from the NVS */
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
        err = nvs_get_i16(my_handle, "low_temp", (int16_t*)&low_temp);
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
        err = nvs_get_i16(my_handle, "high_temp", (int16_t*)&high_temp);
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
        size_t ssid_length = 0;
        err = nvs_get_u8(my_handle, "ssid_length", (uint8_t*)&ssid_length);
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
        err = nvs_get_str(my_handle, "wifi_ssid", (char*)esp_wifi_ssid, &ssid_length);
        switch (err) {
            case ESP_OK:
                printf("Done\n");
                printf("WIFI SSID = %s\n", esp_wifi_ssid);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet!\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

        printf("Reading length of wifi pass from NVS ... ");
        size_t pass_length = 0;
        err = nvs_get_u8(my_handle, "pass_length", (uint8_t*)&pass_length);
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
        err = nvs_get_str(my_handle, "wifi_pass", (char*)esp_wifi_pass, &pass_length);
        switch (err) {
            case ESP_OK:
                printf("Done\n");
                printf("WIFI PASSWORD = %s\n", esp_wifi_pass);
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
/* Store the setting variables to the NVS */
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
        uint8_t ssid_length = sizeof(esp_wifi_ssid);
        err = nvs_set_u8(my_handle, "ssid_length", ssid_length);
        if (err != ESP_OK) return err;
        else{
            printf("Done\n");
        }

        printf("Set wifi ssid from NVS ... ");
        err = nvs_set_str(my_handle, "wifi_ssid", (char*)esp_wifi_ssid);
        if (err != ESP_OK) return err;
        else {
            printf("Done\n");
            printf("WIFI SSID = %s\n", esp_wifi_ssid);
        }

        printf("Set length of wifi pass from NVS ... ");
        uint8_t pass_length = sizeof(esp_wifi_pass);
        err = nvs_set_u8(my_handle, "pass_length", pass_length);
        if (err != ESP_OK) return err;
        else{
            printf("Done\n");
        }

        printf("Set wifi password from NVS ... ");
        err = nvs_set_str(my_handle, "wifi_pass", (char*)esp_wifi_pass);
        if (err != ESP_OK) return err;
        else {
            printf("Done\n");
            printf("WIFI PASSWORD = %s\n", esp_wifi_pass);
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

/* Create the String sending to the server */
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
	ESP_LOGI(">> PUBLISH STRING ", "%s",my_json_string);
	cJSON_Delete(root);
}

/* Parse the String from MQTT server */
void parseString(char *str)
{
    cJSON *json = cJSON_Parse(str);
    if(cJSON_GetObjectItem(json, "cmd")){
        int cmd = cJSON_GetObjectItem(json, "cmd")->valueint;
        ESP_LOGI(">> MQTT COMMAND ", "received command from server, value = %d", cmd);
        // run the motor
        status = cmd;
    }
    if(cJSON_GetObjectItem(json, "auto")){
        auto_mode = cJSON_GetObjectItem(json, "auto")->valueint;
        ESP_LOGI(">> MQTT COMMAND ", "switched mode by server, value = %d", auto_mode);
        // switch auto_mode
    }
    if(cJSON_GetObjectItem(json, "low_temp")){
        low_temp = cJSON_GetObjectItem(json, "low_temp")->valuedouble;
        ESP_LOGI(">> MQTT COMMAND ", "updated low temperature, value = %f", low_temp);
    }
    if(cJSON_GetObjectItem(json, "high_temp")){
        high_temp = cJSON_GetObjectItem(json, "high_temp")->valuedouble;
        ESP_LOGI(">> MQTT COMMAND ", "updated high temperature, value = %f", high_temp);
    }
    cJSON_Delete(json);
}
/* update the setting variables from the String*/
int parseConfig(char *str)
{
    cJSON *conf = cJSON_Parse(str);
    char *my_json_string = cJSON_Print(conf);
	ESP_LOGI(">> APP COMMAND ", "%s",my_json_string);

    if(cJSON_GetObjectItem(conf, "ssid")){
        char *wifi_ssid = cJSON_GetObjectItem(conf, "ssid")->valuestring;
        ESP_LOGI(">> APP COMMAND ", "WiFi SSID = %s", wifi_ssid);
        memcpy(esp_wifi_ssid, wifi_ssid, 32);
    }
    else{
        ESP_LOGE(">> APP COMMAND ", "The setting data does not have the WiFi ssid.");
        cJSON_Delete(conf);
        return ESP_ERR_NOT_FOUND;
    }
    if(cJSON_GetObjectItem(conf, "pass")){
        char *wifi_pass = cJSON_GetObjectItem(conf, "pass")->valuestring;
        ESP_LOGI(">> APP COMMAND ", "WIFI PASSWORD = %s", wifi_pass);
        memcpy(esp_wifi_pass, wifi_pass, 64);
    }
    else{
        ESP_LOGE(">> APP COMMAND ", "The setting data does not have the WiFi password.");
        cJSON_Delete(conf);
        return ESP_ERR_NOT_FOUND;
    }
    
    /*if(cJSON_GetObjectItem(conf, "auto")){
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
    */
    setConfigFromNVS();
    cJSON_Delete(conf);
    return ESP_OK;
}