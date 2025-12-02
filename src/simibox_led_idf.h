/* simibox_led_idf.h - Pure ESP-IDF LED Control */
#ifndef SIMIBOX_LED_IDF_H
#define SIMIBOX_LED_IDF_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/ledc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LED_R_PIN 13  // Moved from GPIO14 (now used by SDMMC CLK)
#define LED_G_PIN 27
#define LED_B_PIN 12

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_12_BIT
#define LEDC_FREQUENCY          5000
#define LEDC_MAX_DUTY           ((1 << LEDC_DUTY_RES) - 1)

#define LEDC_CHANNEL_R          LEDC_CHANNEL_0
#define LEDC_CHANNEL_G          LEDC_CHANNEL_1
#define LEDC_CHANNEL_B          LEDC_CHANNEL_2

esp_err_t led_init(void);
void led_set_color(uint16_t r, uint16_t g, uint16_t b);
void led_show_progress(uint8_t percent);
void led_show_wifi_connecting(void);
void led_show_downloading(void);
void led_show_success(void);
void led_show_error(void);
void led_show_retry(void);
void led_update(void);

#ifdef __cplusplus
}
#endif

#endif /* SIMIBOX_LED_IDF_H */