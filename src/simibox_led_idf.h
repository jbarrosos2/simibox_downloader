/* simibox_led_idf.h - Pure ESP-IDF LED Control */
#ifndef SIMIBOX_LED_IDF_H
#define SIMIBOX_LED_IDF_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/ledc.h"

#ifdef __cplusplus
extern "C" {
#endif

// Hardware pins - must match schematic
#define LED_R_PIN 14
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

// Raw duty write.  If an animation pattern is active the next frame (<=10 ms)
// overwrites it — use led_show_solid() for a color that must stay put.
void led_set_color(uint16_t r, uint16_t g, uint16_t b);

// Static color: cancels the running pattern, then writes the duties.
void led_show_solid(uint16_t r, uint16_t g, uint16_t b);

// Status patterns
void led_show_wifi_connecting(void);  // Purple breathing
void led_show_downloading(void);      // Rainbow (constant speed) - legacy
void led_show_download_progress(uint8_t percent);  // NEW: Rainbow that ACCELERATES!
void led_show_success(void);          // Solid green
void led_show_error(void);            // Slow blinking red
void led_show_retry(void);            // Fast blinking red

// Legacy - color changes with progress (red→orange→yellow→green)
void led_show_progress(uint8_t percent);

// Animation task (Core 1, 100 Hz).  Started automatically by led_init().
// Without it the animation only advances when someone calls led_update(),
// which starves it during blocking I/O (esp_http_client_read fills 1.5 MB
// per call, so the "rainbow" only got ~1 frame every 3 seconds).
esp_err_t led_start_task(void);
void led_stop_task(void);

// Steps the animation one frame.  No-op once the animation task is running
// (the task is then the sole driver); kept for callers that run before
// led_init() or if task creation failed.
void led_update(void);

#ifdef __cplusplus
}
#endif

#endif /* SIMIBOX_LED_IDF_H */