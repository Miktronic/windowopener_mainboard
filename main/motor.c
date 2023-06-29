#include "motor.h"

/* Motor Functions */
void set_motor_dir(int direction) 
{
    printf("Move Up\n");
    gpio_set_level(ENB_OUTPUT_IO, MOTOR_DISABLE); //stop motor

    vTaskDelay(500 / portTICK_PERIOD_MS); //Delay 500ms   
    gpio_set_level(DIR_OUTPUT_IO, direction); 
}

void set_motor_enb(int val)
{
    if(!val){
        printf("Start Motor\n");
    }
    else{
        printf("Stop Motor\n");
    }
    
    gpio_set_level(ENB_OUTPUT_IO, val);
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

void clk_pwm_init(void)
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

void init_motor_gpio(void)
{
    printf("Initiate DIR, ENB pin to Output\n");
    gpio_reset_pin(DIR_OUTPUT_IO);
    gpio_reset_pin(ENB_OUTPUT_IO);
    gpio_reset_pin(MOTOR_CFG0);
    gpio_reset_pin(MOTOR_CFG1);

    /* Set the GPIO as a push/pull output */
    gpio_set_direction(DIR_OUTPUT_IO, GPIO_MODE_OUTPUT);
    gpio_set_direction(ENB_OUTPUT_IO, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_CFG0, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_CFG1, GPIO_MODE_OUTPUT);

    gpio_set_level(DIR_OUTPUT_IO, MOTOR_UP);
    gpio_set_level(ENB_OUTPUT_IO, MOTOR_DISABLE);
    
    // Set 64 steps in TMC2209
    
    gpio_set_level(MOTOR_CFG0, 0);
    gpio_set_level(MOTOR_CFG1, 1);

}