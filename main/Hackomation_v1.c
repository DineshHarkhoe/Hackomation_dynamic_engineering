#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_err.h"
#include "esp_task_wdt.h"
#include "esp_log.h"

#include "driver/i2c.h"
#include "driver/mcpwm.h"

#include "../components/ahrs/MadgwickAHRS.h"
#include "../components/mpu9250/mpu9250.h"
#include "../components/mpu9250/calibrate.h"
#include "../components/mpu9250/common.h"
#include "../components/mpu9250/i2c-easy.h"

static const char *TAG = "Drone";

/*
 * GPIO pin definitions
 */
#define BACK_RIGHT_CW_GPIO 9
#define BACK_LEFT_CCW_GPIO 10
#define FRONT_LEFT_CW_GPIO 2
#define FRONT_RIGHT_CCW_GPIO 4
#define I2C_MASTER_NUM 0 /*!< I2C port number for master dev */

calibration_t cal = {
    .mag_offset = {.x = -31594.468750, .y = 5600.710938, .z = -4488.644531},
    .mag_scale = {.x = 1.513449, .y = 1.557811, .z = 1.434039},
    .accel_offset = {.x = 0.104225, .y = 0.090975, .z = -0.046635},

    .accel_scale_lo = {.x = 1.062676, .y = 1.047455, .z = 0.998575},
    .accel_scale_hi = {.x = -0.931153, .y = -0.948881, .z = -1.018992},

    .gyro_bias_offset = {.x = -2.231220, .y = 0.400365, .z = -0.380849}
};

/**
 * Transformation:
 *  - Rotate around Z axis 180 degrees
 *  - Rotate around X axis -90 degrees
 * @param  {object} s {x,y,z} sensor
 * @return {object}   {x,y,z} transformed
 */
static void transform_accel_gyro(vector_t *v)
{
  float x = v->x;
  float y = v->y;
  float z = v->z;

  v->x = -x;
  v->y = -z;
  v->z = -y;
}

/**
 * Transformation: to get magnetometer aligned
 * @param  {object} s {x,y,z} sensor
 * @return {object}   {x,y,z} transformed
 */
static void transform_mag(vector_t *v)
{
  float x = v->x;
  float y = v->y;
  float z = v->z;

  v->x = -y;
  v->y = z;
  v->z = -x;
}

void get_mpu9265(float *heading, float *pitch, float *roll, float *temp){

    vector_t va, vg, vm;

    // Get the Accelerometer, Gyroscope and Magnetometer values.
    ESP_ERROR_CHECK(get_accel_gyro_mag(&va, &vg, &vm));

    // Transform these values to the orientation of our device.
    transform_accel_gyro(&va);
    transform_accel_gyro(&vg);
    transform_mag(&vm);

    // Apply the AHRS algorithm
    MadgwickAHRSupdate(DEG2RAD(vg.x), DEG2RAD(vg.y), DEG2RAD(vg.z),
                       va.x, va.y, va.z,
                       vm.x, vm.y, vm.z);

    ESP_ERROR_CHECK(get_temperature_celsius(temp));

    MadgwickGetEulerAnglesDegrees(heading, pitch, roll);
}

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
    i2c_mpu9250_init(&cal);
    MadgwickAHRSinit(SAMPLE_FREQ_Hz, 0.8);

    float heading, pitch, roll, temp;
    get_mpu9265(&heading, &pitch, &roll, &temp);
    ESP_LOGI(TAG, "heading: %2.3f°, pitch: %2.3f°, roll: %2.3f°, Temp %2.3f°C", heading, pitch, roll, temp);
}
