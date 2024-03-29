#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

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
#include "driver/gpio.h"

#include "../components/ahrs/MadgwickAHRS.h"
#include "../components/mpu9250/mpu9250.h"
#include "../components/mpu9250/calibrate.h"
#include "../components/mpu9250/common.h"
#include "../components/mpu9250/i2c-easy.h"
#include "../components/mpu9250/bmp280.h"
#include "../components/nmea/nmea_parser.h"
#include "../components/pid/pid_ctrl.h"

static const char *TAG = "Drone";

//15    FR      //1153
//12    BL      //1140
//13    BR      //1140
//2     FL      //1011

#define MOTOR_OFFSET_FL             128
#define MOTOR_OFFSET_FR             130
#define MOTOR_OFFSET_BL             1
#define MOTOR_OFFSET_BR             143

#define FRONT_LEFT_CW_GPIO          2
#define FRONT_RIGHT_CCW_GPIO        15
#define BACK_LEFT_CCW_GPIO          12
#define BACK_RIGHT_CW_GPIO          13

#define TXD_PIN_APPLICATION         27
#define RXD_PIN_APPLICATION         35

#define I2C_MASTER_SCL_IO           19        /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO           18        /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0 /*!< I2C port number for master dev */

#define THREAD_PRIORITY_NORMAL      5
#define THREAD_PRIORITY_LOW         3
#define THREAD_PRIORITY_HIGH        7

#define HC_SR04_SAMPLE_PERIOD_MS    100
#define HC_SR04_PIN_ECHO            GPIO_NUM_32
#define HC_SR04_PIN_TRIG            GPIO_NUM_33
#define HC_SR04_PIN_FRONT_ECHO      GPIO_NUM_25
#define HC_SR04_PIN_FRONT_TRIG      GPIO_NUM_26

#define TIME_ZONE                   (-3)   //Suriname Time zone
#define YEAR_BASE                   (2000) //date in GPS starts from 2000
#define TRIGGER_THREAD_STACK_SIZE   1024
#define RX_BUF_SIZE                 100

_Static_assert(HC_SR04_SAMPLE_PERIOD_MS > 50, "Sample period too short!");

typedef struct {
    uint32_t capture_signal;
    mcpwm_capture_signal_t sel_cap_signal;
} capture;

static uint32_t cap_val_begin_of_sample = 0;
static uint32_t cap_val_end_of_sample = 0;
uint8_t num_satsRead;
float pressureRead = 0.0,
        temperatureRead = 0.0,
        barometric_heightRead = 0.0,
        headingRead = 0.0,
        PitchRead = 0.0,
        rollRead = 0.0,
        hcsr04Read = 0.0,
        latRead = 0.0,
        lonRead = 0.0,
        gpsAltRead = 0.0,
        speedRead = 0.0,
        reference_pitch = 0.0,
        reference_roll = 94.0,
        reference_height = 15.0;
int torqueInPWM = 0,
    rollCorrectionInPWM = 0,
    pitchCorrectionInPWM = 0,
    yawCorrectionInPWM = 0;
bool kill_motors = true;

gps_t *gps = NULL;

static QueueHandle_t cap_queue;
esp_websocket_client_handle_t client;

calibration_t cal = {
    .mag_offset = {.x = 25.183594, .y = 57.519531, .z = -62.648438},
    .mag_scale = {.x = 1.513449, .y = 1.557811, .z = 1.434039},

    .accel_offset = {.x = 0.104225, .y = 0.090975, .z = -0.046635},
    .accel_scale_lo = {.x = 1.062676, .y = 1.047455, .z = 0.998575},
    .accel_scale_hi = {.x = -0.931153, .y = -0.948881, .z = -1.018992},

    .gyro_bias_offset = {.x = -2.231220, .y = 0.400365, .z = -0.380849}
};

//-------------function declaration. TODO: move to header file and make other declarations there
void assignValues(char *dataString);
// ------------end of FUNC DECL

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

void init_rf_comm(void) {
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, TXD_PIN_APPLICATION, RXD_PIN_APPLICATION, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void rx_task(void *arg)
{
    init_rf_comm();
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    char dataStr[10];
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            sprintf(dataStr, "%s", (char *)data);
            assignValues(&dataStr);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    free(data);
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

/*
 * breaks  up string from websocket/RF and assigns important flight variables
 */
void assignValues(char *dataString){
        char *receivedValue;
        strtok_r(dataString, "-", &receivedValue);
        uint8_t dataInt = atoi(dataString);
        if (receivedValue == NULL){
            return;
        }

        printf("integer %d\n", dataInt);
        printf("value in string: %s\n", receivedValue);
        switch(dataInt){
            case 0:
                kill_motors = atoi(receivedValue);
                //printf("debug in case 00");
                break;
            case 1:
                torqueInPWM = atoi(receivedValue);
                //printf("debug in case 01");
                break;
            case 10:
                reference_height = atof(receivedValue);
                break;
            default:
                break;
        }
        printf("kill %d\n", kill_motors);
        printf("torqueInPWM: %d\n", torqueInPWM);
        printf("height referenceede: %f\n", reference_height);
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

void post_to_websocket(void *args){

    while(true){
        //init empty arrays
        char str[300] = "{\"heading\": ";
        char headingStr[10];
        char pitchStr[10];
        char rollStr[10];
        char hcsr04AltStr[10];
        char barometricStr[10];
        char temperature[10];
        char latitudeStr[12];
        char longitudeStr[12];
        char speedStr[10];
        char gps_altStr[10];
        char num_of_satelitesStr[5];
        //char hcsr04AltFrontStr[10];

        //convert float to string
        sprintf(headingStr, "%2.3f", headingRead);
        sprintf(pitchStr, "%2.3f", PitchRead);
        sprintf(rollStr, "%2.3f", rollRead);
        sprintf(hcsr04AltStr, "%2.3f", hcsr04Read);
        sprintf(barometricStr, "%2.3f", barometric_heightRead);
        sprintf(temperature, "%2.3f", temperatureRead);
        sprintf(latitudeStr, "%f", latRead);
        sprintf(longitudeStr, "%f", lonRead);
        sprintf(speedStr, "%2.3f", speedRead);
        sprintf(num_of_satelitesStr, "%d", num_satsRead);
        sprintf(gps_altStr, "%2.3f", gpsAltRead);

        //make hardcoded JSON string to send
        strcat(str, headingStr);
        strcat(str, ", \"pitch\": ");
        strcat(str, pitchStr);
        strcat(str, ", \"roll\": ");
        strcat(str, rollStr);
        strcat(str, ", \"HCSR04\": ");
        strcat(str, hcsr04AltStr);
        strcat(str, ", \"baro_alt\": ");
        strcat(str, barometricStr);
        strcat(str, ", \"gps_alt\": ");
        strcat(str, gps_altStr);
        strcat(str, ", \"temp\": ");
        strcat(str, temperature);
        strcat(str, ", \"lat\": ");
        strcat(str, latitudeStr);
        strcat(str, ", \"lon\": ");
        strcat(str, longitudeStr);
        strcat(str, ", \"velocity\": ");
        strcat(str, speedStr);
        strcat(str, ", \"num_sats\": ");
        strcat(str, num_of_satelitesStr);
        strcat(str, "}");

        if (esp_websocket_client_is_connected(client)) {
            //ESP_LOGI(TAG, "%s\n\n", str);
            esp_websocket_client_send_text(client, str, strlen(str), portMAX_DELAY);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);

}

/**
 * generate single pulse on Trig pin to activate a new sample
 */
static void gen_trig_output(void *arg) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true) {
        vTaskDelayUntil(&xLastWakeTime, HC_SR04_SAMPLE_PERIOD_MS / portTICK_PERIOD_MS);
        ESP_ERROR_CHECK(gpio_set_level(HC_SR04_PIN_TRIG, 1)); // set high
        //ESP_ERROR_CHECK(gpio_set_level(HC_SR04_PIN_FRONT_TRIG, 1));
        esp_rom_delay_us(10);
        //ESP_ERROR_CHECK(gpio_set_level(HC_SR04_PIN_FRONT_TRIG, 0));
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

void run_hcsr04(){
    ESP_LOGI(TAG, "HC-SR04 example based on capture function from MCPWM");

    // the queue where we read data
    cap_queue = xQueueCreate(1, sizeof(uint32_t));
    if (cap_queue == NULL) {
        ESP_LOGE(TAG, "failed to alloc cap_queue");
        return;
    }

    /* configure Echo pin */
    // set CAP_0 on GPIO
    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_2, HC_SR04_PIN_ECHO));
    // enable pull down CAP0, to reduce noise
    ESP_ERROR_CHECK(gpio_pulldown_en(HC_SR04_PIN_ECHO));
    // enable both edge capture on CAP0
    mcpwm_capture_config_t conf = {
        .cap_edge = MCPWM_BOTH_EDGE,
        .cap_prescale = 1,
        .capture_cb = sr04_echo_isr_handler,
        .user_data = NULL
    };
    ESP_ERROR_CHECK(mcpwm_capture_enable_channel(MCPWM_UNIT_0, MCPWM_SELECT_CAP2, &conf));
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

    // start generating trig signal
    xTaskCreate(gen_trig_output, "gen_trig_output", TRIGGER_THREAD_STACK_SIZE, NULL, THREAD_PRIORITY_NORMAL, NULL);
    ESP_LOGI(TAG, "trig task started");
    // forever loop
    while (true) {
        uint32_t pulse_count;
        // block and wait for new measurement
        xQueueReceive(cap_queue, &pulse_count, portMAX_DELAY);
        uint32_t pulse_width_us = pulse_count * (1000000.0 / rtc_clk_apb_freq_get());
        // following formula is based on: https://www.elecrow.com/download/HC_SR04%20Datasheet.pdf
        if (pulse_width_us > 35000) {
            // out of range
            continue;
        }
        hcsr04Read = (float) pulse_width_us / 58;
        //ESP_LOGI(TAG, "Pulse width: %uus, Measured distance: %.2fcm", pulse_width_us, hcsr04Read);
    }
}

void get_altitude_from_pressure(){
    double reference_pressure = 101388;      //Pb    ijkpunt
    double reference_height = 0;             //Hb    ijkpunt
    double molar_mass = 0.0289644;           //M
    double g_constant = 9.80665;             //g
    double R_constant = 8.31432;             //R
    double temp_lapse_rate = -0.0065;        //Lb

    double exponent = (-1 * R_constant * temp_lapse_rate)/(g_constant * molar_mass);
    double pres_frac = pressureRead/reference_pressure;
    pres_frac = pow(pres_frac, exponent);
    pres_frac--;
    double temp_frac = temperatureRead/temp_lapse_rate;

    double height = reference_height + (temp_frac*pres_frac);
    barometric_heightRead = (float)height;
}

void set_bldc_speed(int fr, int fl, int br, int bl){
    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, fl + MOTOR_OFFSET_FL));
    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, fr + MOTOR_OFFSET_FR));
    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, bl + MOTOR_OFFSET_BL));
    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, br + MOTOR_OFFSET_BR));
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
    set_bldc_speed(0, 0 , 0, 0);
    //vTaskDelay(500);          TODO: uncomment delay when not in testing mode
}

/**
 * @brief GPS Event Handler
 *
 * @param event_handler_arg handler specific arguments
 * @param event_base event base, here is fixed to ESP_NMEA_EVENT
 * @param event_id event id
 * @param event_data event specific arguments
 */
static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data){
    switch (event_id) {
    case GPS_UPDATE:
        gps = (gps_t *)event_data;
        /* print information parsed from GPS statements */
/*        ESP_LOGI(TAG, "%d/%d/%d %d:%d:%d => \r\n"
                 "\t\t\t\t\t\tlatitude   = %.05f°N\r\n"
                 "\t\t\t\t\t\tlongitude = %.05f°E\r\n"
                 "\t\t\t\t\t\taltitude   = %.02fm\r\n"
                 "\t\t\t\t\t\tspeed      = %fm/s",
                 gps->date.year + YEAR_BASE, gps->date.month, gps->date.day,
                 gps->tim.hour + TIME_ZONE, gps->tim.minute, gps->tim.second,
                 gps->latitude, gps->longitude, gps->altitude, gps->speed);*/
        latRead = gps->latitude;
        lonRead = gps->longitude;
        gpsAltRead = gps->altitude;
        num_satsRead = gps->sats_in_use;
        speedRead = gps->speed;
        break;
    case GPS_UNKNOWN:
        /* print unknown statements */
        ESP_LOGW(TAG, "Unknown statement:%s", (char *)event_data);
        break;
    default:
        break;
    }
}

void run_gps(void *args){
    /* NMEA parser configuration */
    nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();
    /* init NMEA parser library */
    nmea_parser_handle_t nmea_hdl = nmea_parser_init(&config);

    /* register event handler for NMEA parser library */
    nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL);
    while (true) {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }

    /* unregister event handler */
    nmea_parser_remove_handler(nmea_hdl, gps_event_handler);
    /* deinit NMEA parser library */
    nmea_parser_deinit(nmea_hdl);
    vTaskDelete(NULL);
}

void run_sensor(void *args){
    i2c_mpu9250_init(&cal);
    MadgwickAHRSinit(SAMPLE_FREQ_Hz, 0.8);

    bmp280_params_t params;
    bmp280_init_default_params(&params);
    bmp280_t dev;
    memset(&dev, 0, sizeof(bmp280_t));
    ESP_ERROR_CHECK(bmp280_init(&dev, &params));
    bool bme280p = dev.id == BME280_CHIP_ID;
    printf("BMP280: found %s\n", bme280p ? "BME280" : "BMP280");

    uint64_t i = 0;
    while (true)
    {
        float humidity;

        vector_t va, vg, vm;

        // Get the Accelerometer, Gyroscope and Magnetometer values.
        ESP_ERROR_CHECK(get_accel_gyro_mag(&va, &vg, &vm));
        bmp280_read_float(&dev, &temperatureRead, &pressureRead, &humidity);
        get_altitude_from_pressure();

        // Transform these values to the orientation of our device.
        transform_accel_gyro(&va);
        transform_accel_gyro(&vg);
        transform_mag(&vm);

        // Apply the AHRS algorithm
        MadgwickAHRSupdate(DEG2RAD(vg.x), DEG2RAD(vg.y), DEG2RAD(vg.z),
                        va.x, va.y, va.z,
                        vm.x, vm.y, vm.z);

        //float heading, pitch, roll;
        MadgwickGetEulerAnglesDegrees(&headingRead, &PitchRead, &rollRead);

        // Print the data out every 100 items
        if (i++ % 500 == 0)
        {

            //ESP_LOGI(TAG, "heading: %2.3f°, pitch: %2.3f°, roll: %2.3f°", headingRead, PitchRead, rollRead);

            //Make the WDT happy
            //esp_task_wdt_reset();
        }
        //printf("mpu9265\n");      //debug; uncomment to see whenever tasks runs
        pauses();
    }
    vTaskDelete(NULL);

}

/*
static void sensor_task(void *arg)
{

#ifdef CONFIG_CALIBRATION_MODE
  calibrate_gyro();
  calibrate_accel();
  calibrate_mag();
#else
  run_sensor();
#endif

  // Exit
  //vTaskDelay(100 / portTICK_PERIOD_MS);
  i2c_driver_delete(I2C_MASTER_NUM);

  vTaskDelete(NULL);
}*/

static void run_motors(void *args){
    int mma1 = 0, mma2 = 0, mma3 = 0, mma4 = 0;
    float pitchError, pitchCorrection,
            rollError, rollCorrection,
            heightError, heightCorrection,
            reference_pitch = 0.0,
            reference_roll = 90.0,
            reference_height = 10.0;

    pid_ctrl_parameter_t pitchParam = {
        .kp = 0.2,                      //0.8
        .ki = 0.0,                    //0.002
        .kd = 0.,                      //0.5
        .max_output = 50,
        .min_output = -50,
        .max_integral = 20,
        .min_integral = -20,
        .cal_type = PID_CAL_TYPE_POSITIONAL
    };

    pid_ctrl_parameter_t rollParam = {
        .kp = 0.2,                      //0.8
        .ki = 0.0,                    //0.002
        .kd = 0.,                      //0.3
        .max_output = 50,
        .min_output = -50,
        .max_integral = 20,
        .min_integral = -20,
        .cal_type = PID_CAL_TYPE_POSITIONAL
    };

    pid_ctrl_parameter_t heightParam = {
        .kp = 0.011,
        .ki = 0.002,
        .kd = 0.3,
        .max_output = 10,
        .min_output = -10,
        .max_integral = 0.05,
        .min_integral = -0.05,
        .cal_type = PID_CAL_TYPE_POSITIONAL
    };

    pid_ctrl_config_t pid_ctrl_config_pitch = {.init_param = pitchParam};
    pid_ctrl_config_t pid_ctrl_config_roll = {.init_param = rollParam};
    pid_ctrl_config_t pid_ctrl_config_height = {.init_param = heightParam};

    pid_ctrl_block_handle_t pid_ctrl_block_handle_pitch;
    pid_ctrl_block_handle_t pid_ctrl_block_handle_roll;
    pid_ctrl_block_handle_t pid_ctrl_block_handle_height;

    pid_new_control_block(&pid_ctrl_config_pitch, &pid_ctrl_block_handle_pitch);
    pid_new_control_block(&pid_ctrl_config_roll, &pid_ctrl_block_handle_roll);
    pid_new_control_block(&pid_ctrl_config_height, &pid_ctrl_block_handle_height);

    mcpwm_setup();

    while(true){
        pitchError = PitchRead - reference_pitch;
        rollError = rollRead - reference_roll;
        heightError = reference_height - hcsr04Read;;

        if (kill_motors == 1){
            mma1 = 0, mma2 = 0, mma3 = 0, mma4 = 0;
            pitchCorrection = 0;
            rollCorrection = 0;
            heightCorrection = 0;
            torqueInPWM = 0;
            //ESP_LOGI(TAG, "DIE MOTORS, DIEEEEEEEEEE!!!!!!!!\n");
        } else{
            pid_compute(pid_ctrl_block_handle_pitch, pitchError, &pitchCorrection);
            pid_compute(pid_ctrl_block_handle_roll, rollError, &rollCorrection);
            pid_compute(pid_ctrl_block_handle_height, heightError, &heightCorrection);

            pitchCorrectionInPWM = (int) pitchCorrection;
            rollCorrectionInPWM = (int) rollCorrection;
            //torqueInPWM += (int)heightCorrection;

            if(torqueInPWM < 1000){
                torqueInPWM = 1000;
            } else if (torqueInPWM > 1700){
                torqueInPWM = 1700;
            }

            mma1 = torqueInPWM - rollCorrectionInPWM + pitchCorrectionInPWM + yawCorrectionInPWM;
            mma2 = torqueInPWM + rollCorrectionInPWM + pitchCorrectionInPWM - yawCorrectionInPWM;
            mma3 = torqueInPWM - rollCorrectionInPWM - pitchCorrectionInPWM - yawCorrectionInPWM;
            mma4 = torqueInPWM + rollCorrectionInPWM - pitchCorrectionInPWM + yawCorrectionInPWM;
        }

        set_bldc_speed(mma1, mma2, mma3, mma4);


        //printf("mcpwm\n");    //debug; uncomment to see whenever tasks runs
        //printf("error: %f\n", pitchError);
        //ESP_LOGI(TAG, "Pitch: %f deg\n", PitchRead);
        ESP_LOGI(TAG, "HEIGHT CORRECTION: %f \n", heightCorrection);
        //printf("correctie: %f\n", pitchCorrectionInFloat);
        printf("pwm value: %d\t %d\t %d\t %d\n", mma1, mma2, mma3, mma4);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void app_main(void){

    //vTaskDelay(1000);   //get power level high enough then begin
    websocket_setup();  //gives watchdog error if in websocket task

    xTaskCreate(run_sensor, "MPU9250_task", 4096, NULL, THREAD_PRIORITY_NORMAL, NULL);

    //motor runs on idle priority which is highest priority task can get. make absolutely sure other tasks have sufficient delay
    xTaskCreate(run_motors, "mcpwm_task", 4096, NULL, tskIDLE_PRIORITY, NULL);

    xTaskCreate(run_hcsr04, "HCSR-04_task", 2048, NULL, THREAD_PRIORITY_NORMAL, NULL);
    //xTaskCreate(run_gps, "GPS_task", 2048, NULL, THREAD_PRIORITY_NORMAL, NULL);
    xTaskCreate(post_to_websocket, "websocket_task", 2500, NULL, THREAD_PRIORITY_NORMAL, NULL);
    xTaskCreate(rx_task, "flightApp_task", 3000, NULL, THREAD_PRIORITY_NORMAL, NULL);
}
