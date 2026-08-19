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
#include "freertos/semphr.h"
#include <math.h>

// Animation task: 100 Hz frame rate, pinned to Core 1 (Core 0 runs WiFi/TLS
// and the download loop).  Priority 2 = above the main task (1), well below
// the SD writer (5) and WiFi (23), so it preempts the download loop for the
// few microseconds a frame costs without disturbing throughput.
#define LED_TASK_STACK      3072
#define LED_TASK_PRIORITY   2
#define LED_TASK_CORE       1
#define LED_FRAME_MS        10

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

// Animation task state.  Pattern setters run on the caller's task while the
// animation task reads led_state; every field is a naturally-aligned scalar,
// so writes are atomic on ESP32 and no lock is needed for the state itself.
// The LEDC registers DO need one — see led_set_color().
static TaskHandle_t      s_led_task = NULL;
static SemaphoreHandle_t s_ledc_mutex = NULL;
static volatile bool     s_led_task_stop = false;

static void led_animate_step(void);

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
    // Must exist before the first led_set_color() below.
    if (!s_ledc_mutex) {
        // Recursive: led_animate_step() holds it across a whole frame and the
        // pattern handlers call led_set_color(), which takes it again.
        s_ledc_mutex = xSemaphoreCreateRecursiveMutex();
        if (!s_ledc_mutex) {
            ESP_LOGE(TAG, "Cannot create LEDC mutex");
            return ESP_ERR_NO_MEM;
        }
    }

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

    return led_start_task();
}

void led_set_color(uint16_t r, uint16_t g, uint16_t b) {
    r = (r > LEDC_MAX_DUTY) ? LEDC_MAX_DUTY : r;
    g = (g > LEDC_MAX_DUTY) ? LEDC_MAX_DUTY : g;
    b = (b > LEDC_MAX_DUTY) ? LEDC_MAX_DUTY : b;

    // Serialize the six LEDC register writes: the animation task and the
    // direct callers in downloader_main.cpp can hit them concurrently.
    if (s_ledc_mutex) xSemaphoreTakeRecursive(s_ledc_mutex, portMAX_DELAY);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_R, r);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_G, g);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, b);

    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_R);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_G);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);

    if (s_ledc_mutex) xSemaphoreGiveRecursive(s_ledc_mutex);
}

/**
 * Static color that survives: stops the running animation first.
 *
 * Plain led_set_color() is not enough once the animation task exists — the
 * next frame (<=10 ms later) would repaint whatever pattern is still active.
 * Taking the mutex around both the pattern store and the duty write makes it
 * atomic against a frame in flight.
 */
void led_show_solid(uint16_t r, uint16_t g, uint16_t b) {
    if (s_ledc_mutex) xSemaphoreTakeRecursive(s_ledc_mutex, portMAX_DELAY);
    led_state.pattern = LED_PATTERN_NONE;
    led_set_color(r, g, b);
    if (s_ledc_mutex) xSemaphoreGiveRecursive(s_ledc_mutex);
}

// ═══════════════════════════════════════════════════════════════════════════════
// ANIMATION TASK
// ═══════════════════════════════════════════════════════════════════════════════

static void led_task(void* arg) {
    (void)arg;
    ESP_LOGI(TAG, "LED animation task started on core %d (%d Hz)",
             xPortGetCoreID(), 1000 / LED_FRAME_MS);

    TickType_t last_wake = xTaskGetTickCount();
    while (!s_led_task_stop) {
        led_animate_step();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LED_FRAME_MS));
    }

    ESP_LOGI(TAG, "LED animation task exiting");
    s_led_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t led_start_task(void) {
    if (s_led_task) return ESP_OK;  // Idempotent

    s_led_task_stop = false;
    BaseType_t ret = xTaskCreatePinnedToCore(
        led_task,
        "led_anim",
        LED_TASK_STACK,
        NULL,
        LED_TASK_PRIORITY,
        &s_led_task,
        LED_TASK_CORE
    );

    if (ret != pdPASS) {
        // Not fatal: led_update() still animates when called manually.
        s_led_task = NULL;
        ESP_LOGE(TAG, "Failed to create LED task - falling back to manual led_update()");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

void led_stop_task(void) {
    if (!s_led_task) return;
    s_led_task_stop = true;
    vTaskDelay(pdMS_TO_TICKS(LED_FRAME_MS * 3));  // Let it exit cleanly
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
// ANIMATION UPDATE
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * Legacy entry point.  Once led_task is running it owns the animation, so
 * calls from other tasks are dropped — otherwise every frame would be stepped
 * twice (once by the task, once by the caller) and the rainbow would run at
 * double speed.  Still functional as a fallback if the task never started.
 */
void led_update(void) {
    if (s_led_task && xTaskGetCurrentTaskHandle() != s_led_task) return;
    led_animate_step();
}

static void led_animate_step(void) {
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t elapsed = now - led_state.last_update;

    // Held for the whole frame so led_show_solid() can't be overwritten by a
    // frame that already read the old pattern.
    if (s_ledc_mutex) xSemaphoreTakeRecursive(s_ledc_mutex, portMAX_DELAY);

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

    if (s_ledc_mutex) xSemaphoreGiveRecursive(s_ledc_mutex);
}