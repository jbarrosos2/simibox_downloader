/* simibox_led_idf.c - Pure ESP-IDF LED Control Implementation */
#include "simibox_led_idf.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char* TAG = "led";

typedef enum {
    LED_PATTERN_NONE,
    LED_PATTERN_WIFI_CONNECTING,
    LED_PATTERN_DOWNLOADING,
    LED_PATTERN_SUCCESS,
    LED_PATTERN_ERROR,
    LED_PATTERN_RETRY,
    LED_PATTERN_PROGRESS
} led_pattern_t;

static struct {
    led_pattern_t pattern;
    uint32_t last_update;
    uint32_t phase;
    uint8_t progress_percent;
    bool blink_state;
} led_state = {0};

esp_err_t led_init(void) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    
    ledc_channel_config_t channels[3] = {
        {
            .channel    = LEDC_CHANNEL_R,
            .duty       = 0,
            .gpio_num   = LED_R_PIN,
            .speed_mode = LEDC_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER
        },
        {
            .channel    = LEDC_CHANNEL_G,
            .duty       = 0,
            .gpio_num   = LED_G_PIN,
            .speed_mode = LEDC_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER
        },
        {
            .channel    = LEDC_CHANNEL_B,
            .duty       = 0,
            .gpio_num   = LED_B_PIN,
            .speed_mode = LEDC_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER
        }
    };
    
    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(ledc_channel_config(&channels[i]));
    }
    
    led_set_color(0, 0, 0);
    
    ESP_LOGI(TAG, "LED initialized");
    return ESP_OK;
}

void led_set_color(uint16_t r, uint16_t g, uint16_t b) {
    r = (r > LEDC_MAX_DUTY) ? LEDC_MAX_DUTY : r;
    g = (g > LEDC_MAX_DUTY) ? LEDC_MAX_DUTY : g;
    b = (b > LEDC_MAX_DUTY) ? LEDC_MAX_DUTY : b;
    
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_R, r);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_G, g);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, b);
    
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_R);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_G);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);
}

void led_show_progress(uint8_t percent) {
    led_state.pattern = LED_PATTERN_PROGRESS;
    led_state.progress_percent = percent;
    
    if (percent >= 100) {
        led_set_color(0, LEDC_MAX_DUTY, 0);
        led_state.pattern = LED_PATTERN_SUCCESS;
    }
}

void led_show_wifi_connecting(void) {
    led_state.pattern = LED_PATTERN_WIFI_CONNECTING;
    led_state.phase = 0;
}

void led_show_downloading(void) {
    led_state.pattern = LED_PATTERN_DOWNLOADING;
    led_state.phase = 0;
}

void led_show_success(void) {
    led_state.pattern = LED_PATTERN_SUCCESS;
    led_set_color(0, LEDC_MAX_DUTY, 0);
}

void led_show_error(void) {
    led_state.pattern = LED_PATTERN_ERROR;
    led_state.blink_state = false;
}

void led_show_retry(void) {
    led_state.pattern = LED_PATTERN_RETRY;
    led_state.blink_state = false;
}

void led_update(void) {
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t elapsed = now - led_state.last_update;
    
    switch (led_state.pattern) {
        case LED_PATTERN_WIFI_CONNECTING: {
            if (elapsed >= 10) {
                led_state.last_update = now;
                led_state.phase += 10;
                if (led_state.phase >= 2000) led_state.phase = 0;
                
                float brightness = (sin(2.0f * M_PI * led_state.phase / 2000.0f) + 1.0f) / 2.0f;
                uint16_t duty = (uint16_t)(LEDC_MAX_DUTY * brightness);
                led_set_color(duty, 0, duty);
            }
            break;
        }
        
        case LED_PATTERN_PROGRESS: {
            if (elapsed >= 10) {
                led_state.last_update = now;
                
                uint32_t cycle_time = 2000 - (led_state.progress_percent * 15);
                
                led_state.phase += 10;
                if (led_state.phase >= cycle_time) led_state.phase = 0;
                
                float brightness = (sin(2.0f * M_PI * led_state.phase / cycle_time) + 1.0f) / 2.0f;
                uint16_t duty = (uint16_t)(LEDC_MAX_DUTY * brightness);
                
                if (led_state.progress_percent < 25) {
                    led_set_color(duty, 0, 0);
                } else if (led_state.progress_percent < 50) {
                    led_set_color(duty, duty/2, 0);
                } else if (led_state.progress_percent < 75) {
                    led_set_color(duty, duty, 0);
                } else {
                    led_set_color(0, duty, 0);
                }
            }
            break;
        }
        
        case LED_PATTERN_ERROR: {
            if (elapsed >= 200) {
                led_state.last_update = now;
                led_state.blink_state = !led_state.blink_state;
                led_set_color(led_state.blink_state ? LEDC_MAX_DUTY : 0, 0, 0);
            }
            break;
        }
        
        case LED_PATTERN_RETRY: {
            if (elapsed >= 100) {
                led_state.last_update = now;
                led_state.blink_state = !led_state.blink_state;
                led_set_color(led_state.blink_state ? LEDC_MAX_DUTY : 0, 0, 0);
            }
            break;
        }
        
        case LED_PATTERN_SUCCESS:
        case LED_PATTERN_NONE:
        default:
            break;
    }
}