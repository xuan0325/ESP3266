#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_camera.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "V53_BOT";

// OV2640 DVP
#define CAM_PIN_PWDN    -1
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK    16
#define CAM_PIN_SIOD    17
#define CAM_PIN_SIOC    18
#define CAM_PIN_D7      11
#define CAM_PIN_D6      10
#define CAM_PIN_D5      9
#define CAM_PIN_D4      8
#define CAM_PIN_D3      7
#define CAM_PIN_D2      6
#define CAM_PIN_D1      5
#define CAM_PIN_D0      4
#define CAM_PIN_VSYNC   15
#define CAM_PIN_HREF    14
#define CAM_PIN_PCLK    13

// L293D
#define MOTOR_IN1   1
#define MOTOR_IN2   2
#define MOTOR_IN3   47
#define MOTOR_IN4   48
#define MOTOR_EN1   38
#define MOTOR_EN2   44

// SG90
#define SERVO_GPIO  21

// Soft I2C for PCF8574
#define SW_I2C_SDA 17
#define SW_I2C_SCL 18
#define PCF8574_ADDR 0x20

static void swi2c_sda_out(void)
{
    gpio_set_direction(SW_I2C_SDA, GPIO_MODE_OUTPUT);
}
static void swi2c_sda_in(void)
{
    gpio_set_direction(SW_I2C_SDA, GPIO_MODE_INPUT);
}
static void swi2c_sda_set(uint8_t lvl)
{
    gpio_set_level(SW_I2C_SDA, lvl);
}
static void swi2c_scl_set(uint8_t lvl)
{
    gpio_set_level(SW_I2C_SCL, lvl);
}
static uint8_t swi2c_read_sda(void)
{
    return gpio_get_level(SW_I2C_SDA);
}
static void swi2c_delay(void)
{
    esp_rom_delay_us(2);
}
static void swi2c_start(void)
{
    swi2c_sda_out();
    swi2c_sda_set(1);
    swi2c_scl_set(1);
    swi2c_delay();
    swi2c_sda_set(0);
    swi2c_delay();
    swi2c_scl_set(0);
}
static void swi2c_stop(void)
{
    swi2c_sda_out();
    swi2c_sda_set(0);
    swi2c_scl_set(1);
    swi2c_delay();
    swi2c_sda_set(1);
}
static uint8_t swi2c_write_byte(uint8_t dat)
{
    uint8_t i, ack;
    swi2c_sda_out();
    for(i=0;i<8;i++)
    {
        swi2c_sda_set(dat & 0x80);
        dat <<=1;
        swi2c_delay();
        swi2c_scl_set(1);
        swi2c_delay();
        swi2c_scl_set(0);
    }
    swi2c_sda_in();
    swi2c_delay();
    swi2c_scl_set(1);
    swi2c_delay();
    ack = swi2c_read_sda();
    swi2c_scl_set(0);
    return ack;
}

static esp_err_t pcf8574_write_byte(uint8_t data)
{
    swi2c_start();
    if(swi2c_write_byte((PCF8574_ADDR <<1)|0x00))
    {
        swi2c_stop();
        return ESP_FAIL;
    }
    if(swi2c_write_byte(data))
    {
        swi2c_stop();
        return ESP_FAIL;
    }
    swi2c_stop();
    return ESP_OK;
}

static esp_err_t init_camera(void)
{
    camera_config_t config = {
        .pin_pwdn = CAM_PIN_PWDN,
        .pin_reset = CAM_PIN_RESET,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,
        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href = CAM_PIN_HREF,
        .pin_pclk = CAM_PIN_PCLK,
        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = FRAMESIZE_QVGA,
        .jpeg_quality = 12,
        .fb_count = 1,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    };
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "cam init fail: 0x%x", err);
        return err;
    }
    ESP_LOGI(TAG, "OV2640 init ok");
    return ESP_OK;
}

static void init_motors(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MOTOR_IN1) | (1ULL << MOTOR_IN2) |
                        (1ULL << MOTOR_IN3) | (1ULL << MOTOR_IN4) |
                        (1ULL << MOTOR_EN1) | (1ULL << MOTOR_EN2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(MOTOR_IN1, 0);
    gpio_set_level(MOTOR_IN2, 0);
    gpio_set_level(MOTOR_IN3, 0);
    gpio_set_level(MOTOR_IN4, 0);
    gpio_set_level(MOTOR_EN1, 0);
    gpio_set_level(MOTOR_EN2, 0);
    ESP_LOGI(TAG, "L293D gpio ready");
}

static void init_servo(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = 50,
    };
    ledc_timer_config(&timer);
    ledc_channel_config_t ch = {
        .gpio_num = SERVO_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&ch);
    ESP_LOGI(TAG, "SG90 pwm ready GPIO%d", SERVO_GPIO);
}

void servo_set_angle(int angle)
{
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    uint32_t pulse_us = 500 + (angle * 2000 / 180);
    uint32_t duty = (pulse_us * 16384UL) / 20000UL;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

void motor_a_forward(void)
{
    gpio_set_level(MOTOR_IN1, 1);
    gpio_set_level(MOTOR_IN2, 0);
    gpio_set_level(MOTOR_EN1, 1);
}
void motor_a_backward(void)
{
    gpio_set_level(MOTOR_IN1, 0);
    gpio_set_level(MOTOR_IN2, 1);
    gpio_set_level(MOTOR_EN1, 1);
}
void motor_a_stop(void)
{
    gpio_set_level(MOTOR_IN1, 0);
    gpio_set_level(MOTOR_IN2, 0);
    gpio_set_level(MOTOR_EN1, 0);
}

void motor_b_forward(void)
{
    gpio_set_level(MOTOR_IN3, 1);
    gpio_set_level(MOTOR_IN4, 0);
    gpio_set_level(MOTOR_EN2, 1);
}
void motor_b_backward(void)
{
    gpio_set_level(MOTOR_IN3, 0);
    gpio_set_level(MOTOR_IN4, 1);
    gpio_set_level(MOTOR_EN2, 1);
}
void motor_b_stop(void)
{
    gpio_set_level(MOTOR_IN3, 0);
    gpio_set_level(MOTOR_IN4, 0);
    gpio_set_level(MOTOR_EN2, 0);
}

void motor_stop_all(void)
{
    motor_a_stop();
    motor_b_stop();
}
void car_forward(void)
{
    motor_a_forward();
    motor_b_forward();
}
void car_backward(void)
{
    motor_a_backward();
    motor_b_backward();
}
void car_turn_left(void)
{
    motor_a_backward();
    motor_b_forward();
}
void car_turn_right(void)
{
    motor_a_forward();
    motor_b_backward();
}

void camera_task(void *arg)
{
    camera_fb_t *fb = NULL;
    for(;;)
    {
        fb = esp_camera_fb_get();
        if(!fb)
        {
            ESP_LOGE(TAG, "get frame fail");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        ESP_LOGI(TAG, "frame w=%d h=%d len=%d", fb->width, fb->height, fb->len);
        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

void motion_test_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "motion test start");
    servo_set_angle(90);
    for(;;)
    {
        car_forward();
        vTaskDelay(pdMS_TO_TICKS(800));
        motor_stop_all();
        vTaskDelay(pdMS_TO_TICKS(500));

        car_turn_left();
        vTaskDelay(pdMS_TO_TICKS(400));
        motor_stop_all();
        vTaskDelay(pdMS_TO_TICKS(500));

        car_forward();
        vTaskDelay(pdMS_TO_TICKS(800));
        motor_stop_all();
        vTaskDelay(pdMS_TO_TICKS(500));

        car_turn_right();
        vTaskDelay(pdMS_TO_TICKS(400));
        motor_stop_all();
        vTaskDelay(pdMS_TO_TICKS(800));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "V5.3 bot start");
    init_motors();
    init_servo();

    esp_err_t cam_err = init_camera();
    if(cam_err != ESP_OK)
    {
        ESP_LOGE(TAG, "camera init failed");
    }

    pcf8574_write_byte(0xFF);

    if(cam_err == ESP_OK)
    {
        xTaskCreate(camera_task, "cam_task", 4096, NULL, 5, NULL);
    }
    xTaskCreate(motion_test_task, "motion_task", 2048, NULL, 4, NULL);
}
