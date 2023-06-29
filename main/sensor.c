#include "sensor.h"

void init_sensor_gpio(void)
{
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

    ESP_LOGI(">> SENSOR ", "Initiation sensor gpio pins");
}

void scan_sensors(void)
{
    pos_start_trigger = gpio_get_level(GPIO_INPUT_IO_0);
    pos_quarter_trigger = gpio_get_level(GPIO_INPUT_IO_1);
    pos_half_trigger = gpio_get_level(GPIO_INPUT_IO_2);
    pos_third_trigger = gpio_get_level(GPIO_INPUT_IO_3);
    pos_end_trigger = gpio_get_level(GPIO_INPUT_IO_4);
    ESP_LOGI(">> SENSOR VALUES ", "POS(0%%): %d, POS(25%%): %d, POS(50%%): %d, POS(75%%): %d, POS(100%%): %d", 
                pos_start_trigger, pos_quarter_trigger, pos_half_trigger, pos_third_trigger, pos_end_trigger);
}

