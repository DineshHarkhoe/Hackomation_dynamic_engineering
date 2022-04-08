#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "soc/rtc.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_err.h"
#include "esp_task_wdt.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_wifi.h"
#include "esp_websocket_client.h"

#include "driver/i2c.h"
#include "driver/mcpwm.h"

#include "../components/ahrs/MadgwickAHRS.h"
#include "../components/mpu9250/mpu9250.h"
#include "../components/mpu9250/calibrate.h"
#include "../components/mpu9250/common.h"
#include "../components/mpu9250/i2c-easy.h"

static const char *TAG = "Drone";

#define BACK_RIGHT_CW_GPIO 9
#define BACK_LEFT_CCW_GPIO 10
#define FRONT_LEFT_CW_GPIO 2
#define FRONT_RIGHT_CCW_GPIO 4
#define I2C_MASTER_NUM 0

#define HC_SR04_SAMPLE_PERIOD_MS    100
#define HC_SR04_PIN_ECHO    GPIO_NUM_32
#define HC_SR04_PIN_TRIG    GPIO_NUM_33
#define TRIGGER_THREAD_STACK_SIZE 512
#define TRIGGER_THREAD_PRIORITY 5

_Static_assert(HC_SR04_SAMPLE_PERIOD_MS > 50, "Sample period too short!");

typedef struct {
    uint32_t capture_signal;
    mcpwm_capture_signal_t sel_cap_signal;
} capture;

static uint32_t cap_val_begin_of_sample = 0;
static uint32_t cap_val_end_of_sample = 0;

static QueueHandle_t cap_queue;
esp_websocket_client_handle_t client;

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

static void websocket_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
    switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "WEBSOCKET_EVENT_CONNECTED");
        break;
    case WEBSOCKET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "WEBSOCKET_EVENT_DISCONNECTED");
        break;
    case WEBSOCKET_EVENT_DATA:
        //ESP_LOGI(TAG, "WEBSOCKET_EVENT_DATA");
        break;
    case WEBSOCKET_EVENT_ERROR:
        ESP_LOGI(TAG, "WEBSOCKET_EVENT_ERROR");
        break;
    }
}

void websocket_setup(){
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

    esp_websocket_client_config_t websocket_cfg = {};
    websocket_cfg.uri = "ws://192.168.1.15:8080";

    ESP_LOGI(TAG, "Connecting to %s...", websocket_cfg.uri);

    client = esp_websocket_client_init(&websocket_cfg);
    esp_websocket_register_events(client, WEBSOCKET_EVENT_ANY, websocket_event_handler, (void *)client);

    esp_websocket_client_start(client);
}

void post_to_websocket(float *heading, float *pitch, float *roll, float *hcsr04Alt){
    //init empty arrays
    char str[200] = "{\"heading\": ";
    char headingStr[10];
    char pitchStr[10];
    char rollStr[10];
    char hcsr04AltStr[10];

    //convert float to string
    sprintf(headingStr, "%2.3f", *heading);
    sprintf(pitchStr, "%2.3f", *pitch);
    sprintf(rollStr, "%2.3f", *roll);
    sprintf(hcsr04AltStr, "%2.3f", *hcsr04Alt);

    //make hardcoded JSON string to send
    strcat(str, headingStr);
    strcat(str, ", \"pitch\": ");
    strcat(str, pitchStr);
    strcat(str, ", \"roll\": ");
    strcat(str, rollStr);
    strcat(str, ", \"HCSR04\": ");
    strcat(str, hcsr04AltStr);
    strcat(str, "}");

    if (esp_websocket_client_is_connected(client)) {
        //ESP_LOGI(TAG, "Sending %s", str);
        esp_websocket_client_send_text(client, str, strlen(str), portMAX_DELAY);
    }
}

/**
 * generate single pulse on Trig pin to activate a new sample
 */
static void gen_trig_output(void *arg) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true) {
        vTaskDelayUntil(&xLastWakeTime, HC_SR04_SAMPLE_PERIOD_MS / portTICK_PERIOD_MS);
        ESP_ERROR_CHECK(gpio_set_level(HC_SR04_PIN_TRIG, 1)); // set high
        esp_rom_delay_us(10);
        ESP_ERROR_CHECK(gpio_set_level(HC_SR04_PIN_TRIG, 0)); // set low
    }
}

/**
 * this is an ISR callback, we take action according to the captured edge
 */
static bool sr04_echo_isr_handler(mcpwm_unit_t mcpwm, mcpwm_capture_channel_id_t cap_sig, const cap_event_data_t *edata,void *arg) {
    //calculate the interval in the ISR,
    //so that the interval will be always correct even when cap_queue is not handled in time and overflow.
    BaseType_t high_task_wakeup = pdFALSE;
    if (edata->cap_edge == MCPWM_POS_EDGE) {
        // store the timestamp when pos edge is detected
        cap_val_begin_of_sample = edata->cap_value;
        cap_val_end_of_sample = cap_val_begin_of_sample;
    } else {
        cap_val_end_of_sample = edata->cap_value;
        // following formula refers to: https://www.elecrow.com/download/HC_SR04%20Datasheet.pdf
        uint32_t pulse_count = cap_val_end_of_sample - cap_val_begin_of_sample;
        // send measurement back though queue
        xQueueSendFromISR(cap_queue, &pulse_count, &high_task_wakeup);
    }
    return high_task_wakeup == pdTRUE;
}

void get_hcsr04(float *distance){
    uint32_t pulse_count;
    // block and wait for new measurement
    xQueueReceive(cap_queue, &pulse_count, portMAX_DELAY);
    uint32_t pulse_width_us = pulse_count * (1000000.0 / rtc_clk_apb_freq_get());
    // following formula is based on: https://www.elecrow.com/download/HC_SR04%20Datasheet.pdf
    if (pulse_width_us > 35000) {
    }
    *distance = (float) pulse_width_us / 58;
}

void hcsr04_setup(){
    // the queue where we read data
    cap_queue = xQueueCreate(1, sizeof(uint32_t));
    if (cap_queue == NULL) {
        ESP_LOGE(TAG, "failed to alloc cap_queue");
        return;
    }

    /* configure Echo pin */
    // set CAP_0 on GPIO
    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, HC_SR04_PIN_ECHO));
    // enable pull down CAP0, to reduce noise
    ESP_ERROR_CHECK(gpio_pulldown_en(HC_SR04_PIN_ECHO));
    // enable both edge capture on CAP0
    mcpwm_capture_config_t conf = {
        .cap_edge = MCPWM_BOTH_EDGE,
        .cap_prescale = 1,
        .capture_cb = sr04_echo_isr_handler,
        .user_data = NULL
    };
    ESP_ERROR_CHECK(mcpwm_capture_enable_channel(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, &conf));
    ESP_LOGI(TAG, "Echo pin configured");

    /* configure Trig pin */
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pin_bit_mask = BIT64(HC_SR04_PIN_TRIG),
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(HC_SR04_PIN_TRIG, 0)); // drive low by default
    ESP_LOGI(TAG, "Trig pin configured");

    xTaskCreate(gen_trig_output, "gen_trig_output", TRIGGER_THREAD_STACK_SIZE, NULL, TRIGGER_THREAD_PRIORITY, NULL);
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
    //vTaskDelay(3000);   //30sec
}

void set_bldc_speed(int fr, int fl, int br, int bl){
    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, fl));
    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, fr));
    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, bl));
    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, br));

    vTaskDelay(5);
}

void run_sensor(void)
{

    i2c_mpu9250_init(&cal);
    MadgwickAHRSinit(SAMPLE_FREQ_Hz, 0.8);

    uint64_t i = 0;
    while (true)
    {
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

        // Print the data out every 100 items
        if (i++ % 300 == 0)
        {
            float temp;
            ESP_ERROR_CHECK(get_temperature_celsius(&temp));

            float heading, pitch, roll;
            MadgwickGetEulerAnglesDegrees(&heading, &pitch, &roll);
            ESP_LOGI(TAG, "heading: %2.3f°, pitch: %2.3f°, roll: %2.3f°, Temp %2.3f°C", heading, pitch, roll, temp);
            post_to_websocket(&heading, &pitch,  &roll, &temp);

            //Make the WDT happy
            esp_task_wdt_reset();
        }

        pauses();
    }
}

static void sensor_task(/*void *arg*/)
{

#ifdef CONFIG_CALIBRATION_MODE
  calibrate_gyro();
  calibrate_accel();
  calibrate_mag();
#else
  run_sensor();
#endif

  // Exit
  vTaskDelay(100 / portTICK_PERIOD_MS);
  i2c_driver_delete(I2C_MASTER_NUM);

  vTaskDelete(NULL);
}

void app_main(void)
{
    //mcpwm_setup();
    //hcsr04_setup();
    websocket_setup();

    xTaskCreate(sensor_task, "sensor_task", 2048, NULL, 10, NULL);

}
