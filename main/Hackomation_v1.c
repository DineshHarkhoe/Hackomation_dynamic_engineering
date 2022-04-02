
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm.h"

static const char *TAG = "example";

// You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH_US (1000) // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US (2000) // Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE        (90)   // Maximum angle in degree upto which servo can rotate

#define BACK_RIGHT_CW_GPIO 9
#define BACK_LEFT_CCW_GPIO 10
#define FRONT_LEFT_CW_GPIO 2
#define FRONT_RIGHT_CCW_GPIO 4


void mcpwm_setup(){
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, FRONT_LEFT_CW_GPIO);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, FRONT_RIGHT_CCW_GPIO);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, BACK_LEFT_CCW_GPIO);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, BACK_RIGHT_CW_GPIO);

    mcpwm_config_t pwm_config = {
        .frequency = 50, // frequency = 50Hz, i.e. for every servo motor time period should be 20ms
        .cmpr_a = 0,     // duty cycle of PWMxA = 0
        .cmpr_b = 0,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);


    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1000));
    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 1000));
    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 1000));
    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, 1000));
    vTaskDelay(3000);   //30sec
}

void set_bldc_speed(int fr, int fl, int br, int bl){
    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, fl));
    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, fr));
    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, bl));
    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, br));

    vTaskDelay(5);
}

void app_main(void)
{
    mcpwm_setup();


    while (1) {
        for (int p = 1000; p<=1500; p++){
            set_bldc_speed(p, p, p, p);

            if (p == 1500){
                for (p = 1500; p>=1000; p--){
                    set_bldc_speed(p, p, p, p);
                }
            }
        ESP_LOGI(TAG, "duty: %uus", mcpwm_get_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A));

        }
    }
}
