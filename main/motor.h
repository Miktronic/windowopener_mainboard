#ifndef MOTOR_H_
#define MOTOR_H_

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"

#define CLK_TIMER              LEDC_TIMER_0
#define CLK_MODE               LEDC_LOW_SPEED_MODE
#define CLK_OUTPUT_IO          (2) // Define the output GPIO
#define CLK_CHANNEL            LEDC_CHANNEL_0
#define CLK_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define CLK_DUTY               (5733) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define CLK_FREQUENCY          (1000) // Frequency in Hertz. Set frequency at 5 kHz

#define DIR_OUTPUT_IO           (1)
#define ENB_OUTPUT_IO           (3)

#define MOTOR_ENABLE            (0)
#define MOTOR_DISABLE           (1)

#define MOTOR_UP                (0)
#define MOTOR_DOWN              (1)

#define MOTOR_CFG0              (5)
#define MOTOR_CFG1              (4)

void init_motor_gpio(void);
void clk_pwm_init(void);
void set_clk_duty(int duty);
void set_motor_enb(int val);
void set_motor_dir(int direction);

#endif