#ifndef SENSOR_H_
#define SENSOR_H_

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#define GPIO_INPUT_IO_0         6
#define GPIO_INPUT_IO_1         7
#define GPIO_INPUT_IO_2         8
#define GPIO_INPUT_IO_3         9
#define GPIO_INPUT_IO_4         10

#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1) | (1ULL<<GPIO_INPUT_IO_2) | (1ULL<<GPIO_INPUT_IO_3) | (1ULL<<GPIO_INPUT_IO_4))

extern uint8_t pos_start_trigger;
extern uint8_t pos_quarter_trigger;
extern uint8_t pos_half_trigger;
extern uint8_t pos_third_trigger;
extern uint8_t pos_end_trigger;

void init_sensor_gpio(void);
void scan_sensors(void);

#endif