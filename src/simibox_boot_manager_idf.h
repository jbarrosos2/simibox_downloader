/* simibox_boot_manager_idf.h - Pure ESP-IDF Boot Manager */
#ifndef SIMIBOX_BOOT_MANAGER_IDF_H
#define SIMIBOX_BOOT_MANAGER_IDF_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "nvs.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BOOT_REASON_NORMAL = 0,
    BOOT_REASON_DOWNLOAD_REQUEST = 1,
    BOOT_REASON_DOWNLOAD_SUCCESS = 2,
    BOOT_REASON_DOWNLOAD_FAILED = 3,
    BOOT_REASON_FACTORY_RESET = 4
} boot_reason_t;

typedef enum {
    DOWNLOAD_ERROR_NONE = 0,
    DOWNLOAD_ERROR_WIFI_CONNECT = 1,
    DOWNLOAD_ERROR_SERVER_UNREACHABLE = 2,
    DOWNLOAD_ERROR_DOWNLOAD_FAILED = 3,
    DOWNLOAD_ERROR_VERIFICATION_FAILED = 4,
    DOWNLOAD_ERROR_SD_CARD = 5,
    DOWNLOAD_ERROR_TIMEOUT = 6
} download_error_t;

typedef struct {
    boot_reason_t reason;
    char folder[64];
    uint8_t retry_count;
    download_error_t last_error;
    uint8_t volume;
    uint8_t max_volume;
    char last_ssid[32];
    uint32_t download_size;
    uint8_t download_progress;
} boot_state_t;

esp_err_t boot_mgr_init(void);
esp_err_t boot_mgr_save_state(const boot_state_t* state);
esp_err_t boot_mgr_load_state(boot_state_t* state);
esp_err_t boot_mgr_clear_state(void);
esp_err_t boot_mgr_request_download(const char* folder, uint32_t expected_size);
esp_err_t boot_mgr_update_progress(uint8_t percent);
esp_err_t boot_mgr_report_success(void);
esp_err_t boot_mgr_report_failure(download_error_t error);
const char* boot_mgr_get_error_string(download_error_t error);
bool boot_mgr_needs_download(void);
esp_err_t boot_mgr_set_volume(uint8_t vol);
esp_err_t boot_mgr_get_volume(uint8_t* vol);

#ifdef __cplusplus
}
#endif

#endif