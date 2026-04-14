/* simibox_led_idf.c - Pure ESP-IDF LED Control Implementation
 * 
 * Features:
 * - Rainbow download progress that ACCELERATES as download completes
 * - WiFi connecting purple breathing
 * - Success/Error indicators
 */
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
    LED_PATTERN_DOWNLOAD_RAINBOW,  // Accelerating rainbow!
    LED_PATTERN_SUCCESS,
    LED_PATTERN_ERROR,
    LED_PATTERN_RETRY,
    LED_PATTERN_PROGRESS
} led_pattern_t;

static struct {
    led_pattern_t pattern;
    uint32_t last_update;
    float hue;              // 0.0 - 1.0 for rainbow
    uint8_t progress_percent;
    bool blink_state;
} led_state = {0};

// ═══════════════════════════════════════════════════════════════════════════════
// HSV to RGB conversion for smooth rainbow
// ═══════════════════════════════════════════════════════════════════════════════
static void hsv_to_rgb(float h, float s, float v, uint16_t* r, uint16_t* g, uint16_t* b) {
    float c = v * s;
    float x = c * (1.0f - fabsf(fmodf(h * 6.0f, 2.0f) - 1.0f));
    float m = v - c;
    
    float rf, gf, bf;
    
    if (h < 1.0f/6.0f) {
        rf = c; gf = x; bf = 0;
    } else if (h < 2.0f/6.0f) {
        rf = x; gf = c; bf = 0;
    } else if (h < 3.0f/6.0f) {
        rf = 0; gf = c; bf = x;
    } else if (h < 4.0f/6.0f) {
        rf = 0; gf = x; bf = c;
    } else if (h < 5.0f/6.0f) {
        rf = x; gf = 0; bf = c;
    } else {
        rf = c; gf = 0; bf = x;
    }
    
    *r = (uint16_t)((rf + m) * LEDC_MAX_DUTY);
    *g = (uint16_t)((gf + m) * LEDC_MAX_DUTY);
    *b = (uint16_t)((bf + m) * LEDC_MAX_DUTY);
}

// ═══════════════════════════════════════════════════════════════════════════════
// INITIALIZATION
// ═══════════════════════════════════════════════════════════════════════════════

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
    
    ESP_LOGI(TAG, "LED initialized (R=%d, G=%d, B=%d)", LED_R_PIN, LED_G_PIN, LED_B_PIN);
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

// ═══════════════════════════════════════════════════════════════════════════════
// PATTERN SETTERS
// ═══════════════════════════════════════════════════════════════════════════════

void led_show_wifi_connecting(void) {
    led_state.pattern = LED_PATTERN_WIFI_CONNECTING;
    led_state.hue = 0;
}

void led_show_downloading(void) {
    led_state.pattern = LED_PATTERN_DOWNLOADING;
    led_state.hue = 0;
}

/**
 * Accelerating rainbow for download progress!
 * 
 * Visual effect:
 *   0%:   Rainbow cycles slowly (~3 seconds per cycle)
 *   50%:  Rainbow cycles faster (~1 second per cycle)  
 *   100%: Rainbow cycles very fast (~0.3 seconds per cycle)
 * 
 * Plus a subtle pulsing that also accelerates, creating
 * a "heartbeat" effect that gets more excited as download completes.
 */
void led_show_download_progress(uint8_t percent) {
    led_state.pattern = LED_PATTERN_DOWNLOAD_RAINBOW;
    led_state.progress_percent = (percent > 100) ? 100 : percent;
}

void led_show_success(void) {
    led_state.pattern = LED_PATTERN_SUCCESS;
    led_set_color(0, LEDC_MAX_DUTY, 0);  // Solid green
}

void led_show_error(void) {
    led_state.pattern = LED_PATTERN_ERROR;
    led_state.blink_state = false;
}

void led_show_retry(void) {
    led_state.pattern = LED_PATTERN_RETRY;
    led_state.blink_state = false;
}

// Legacy: color changes with progress (red→orange→yellow→green)
void led_show_progress(uint8_t percent) {
    led_state.pattern = LED_PATTERN_PROGRESS;
    led_state.progress_percent = percent;
    
    if (percent >= 100) {
        led_set_color(0, LEDC_MAX_DUTY, 0);
        led_state.pattern = LED_PATTERN_SUCCESS;
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// ANIMATION UPDATE - Call frequently in your main loop!
// ═══════════════════════════════════════════════════════════════════════════════

void led_update(void) {
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t elapsed = now - led_state.last_update;
    
    switch (led_state.pattern) {
        
        // ─────────────────────────────────────────────────────────────────────
        // WiFi Connecting: Purple breathing
        // ─────────────────────────────────────────────────────────────────────
        case LED_PATTERN_WIFI_CONNECTING: {
            if (elapsed >= 20) {
                led_state.last_update = now;
                led_state.hue += 0.01f;
                if (led_state.hue >= 1.0f) led_state.hue = 0;
                
                // Breathing effect
                float brightness = (sinf(led_state.hue * 2.0f * M_PI) + 1.0f) / 2.0f;
                brightness = 0.2f + brightness * 0.8f;  // Min 20% brightness
                
                uint16_t duty = (uint16_t)(LEDC_MAX_DUTY * brightness);
                led_set_color(duty, 0, duty);  // Purple
            }
            break;
        }
        
        // ─────────────────────────────────────────────────────────────────────
        // Downloading: Constant speed rainbow (legacy)
        // ─────────────────────────────────────────────────────────────────────
        case LED_PATTERN_DOWNLOADING: {
            if (elapsed >= 20) {
                led_state.last_update = now;
                
                // Fixed speed: one cycle every ~2 seconds
                led_state.hue += 0.01f;
                if (led_state.hue >= 1.0f) led_state.hue -= 1.0f;
                
                uint16_t r, g, b;
                hsv_to_rgb(led_state.hue, 1.0f, 1.0f, &r, &g, &b);
                led_set_color(r, g, b);
            }
            break;
        }
        
        // ─────────────────────────────────────────────────────────────────────
        // Download Rainbow: ACCELERATES with progress!
        // ─────────────────────────────────────────────────────────────────────
        case LED_PATTERN_DOWNLOAD_RAINBOW: {
            if (elapsed >= 10) {  // 100 Hz update for smooth animation
                led_state.last_update = now;
                
                // Speed calculation (quadratic acceleration):
                //   At 0%:   hue_step = 0.003 → ~3.3 seconds per cycle (slow)
                //   At 50%:  hue_step = 0.013 → ~0.8 seconds per cycle
                //   At 100%: hue_step = 0.043 → ~0.23 seconds per cycle (fast!)
                
                float progress = led_state.progress_percent / 100.0f;
                float hue_step = 0.003f + (progress * progress) * 0.040f;
                
                led_state.hue += hue_step;
                if (led_state.hue >= 1.0f) led_state.hue -= 1.0f;
                
                // Add subtle pulsing that also accelerates
                float pulse_freq = 2.0f + progress * 8.0f;  // 2-10 Hz
                float pulse = (sinf(now * pulse_freq * 0.001f * 2.0f * M_PI) + 1.0f) / 2.0f;
                float brightness = 0.6f + pulse * 0.4f;  // 60-100% brightness
                
                uint16_t r, g, b;
                hsv_to_rgb(led_state.hue, 1.0f, brightness, &r, &g, &b);
                led_set_color(r, g, b);
            }
            break;
        }
        
        // ─────────────────────────────────────────────────────────────────────
        // Legacy Progress: Color changes with progress
        // ─────────────────────────────────────────────────────────────────────
        case LED_PATTERN_PROGRESS: {
            if (elapsed >= 30) {
                led_state.last_update = now;
                
                // Pulse speed increases with progress
                uint32_t cycle_time = 2000 - (led_state.progress_percent * 15);
                if (cycle_time < 200) cycle_time = 200;
                
                float phase = fmodf((float)now, (float)cycle_time) / (float)cycle_time;
                float brightness = (sinf(phase * 2.0f * M_PI) + 1.0f) / 2.0f;
                uint16_t duty = (uint16_t)(LEDC_MAX_DUTY * brightness);
                
                // Color based on progress
                if (led_state.progress_percent < 25) {
                    led_set_color(duty, 0, 0);              // Red
                } else if (led_state.progress_percent < 50) {
                    led_set_color(duty, duty/2, 0);         // Orange
                } else if (led_state.progress_percent < 75) {
                    led_set_color(duty, duty, 0);           // Yellow
                } else {
                    led_set_color(0, duty, 0);              // Green
                }
            }
            break;
        }
        
        // ─────────────────────────────────────────────────────────────────────
        // Error: Slow red blink (300ms)
        // ─────────────────────────────────────────────────────────────────────
        case LED_PATTERN_ERROR: {
            if (elapsed >= 300) {
                led_state.last_update = now;
                led_state.blink_state = !led_state.blink_state;
                led_set_color(led_state.blink_state ? LEDC_MAX_DUTY : 0, 0, 0);
            }
            break;
        }
        
        // ─────────────────────────────────────────────────────────────────────
        // Retry: Fast red blink (100ms)
        // ─────────────────────────────────────────────────────────────────────
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
            // Static color, no animation needed
            break;
    }
}