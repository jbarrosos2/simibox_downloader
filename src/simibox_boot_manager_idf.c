/* simibox_boot_manager_idf.c - Pure ESP-IDF Boot Manager Implementation */
#include "simibox_boot_manager_idf.h"
#include "esp_log.h"
#include <string.h>

static const char* TAG = "boot_mgr";
static const char* NVS_NAMESPACE = "bootmgr";

static const char* KEY_REASON = "reason";
static const char* KEY_FOLDER = "folder";
static const char* KEY_RETRY = "retry";
static const char* KEY_ERROR = "error";
static const char* KEY_VOLUME = "volume";
static const char* KEY_MAX_VOL = "maxvol";
static const char* KEY_SSID = "ssid";
static const char* KEY_SIZE = "size";
static const char* KEY_PROGRESS = "progress";

static nvs_handle_t g_nvs_handle = 0;
static boot_state_t g_current_state = {0};

esp_err_t boot_mgr_init(void) {
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &g_nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(ret));
        return ret;
    }
    
    boot_mgr_load_state(&g_current_state);
    
    ESP_LOGI(TAG, "Boot manager initialized");
    return ESP_OK;
}

esp_err_t boot_mgr_save_state(const boot_state_t* state) {
    if (!state || g_nvs_handle == 0) return ESP_ERR_INVALID_STATE;
    
    esp_err_t ret = ESP_OK;
    
    ret |= nvs_set_u8(g_nvs_handle, KEY_REASON, (uint8_t)state->reason);
    ret |= nvs_set_str(g_nvs_handle, KEY_FOLDER, state->folder);
    ret |= nvs_set_u8(g_nvs_handle, KEY_RETRY, state->retry_count);
    ret |= nvs_set_u8(g_nvs_handle, KEY_ERROR, (uint8_t)state->last_error);
    ret |= nvs_set_u8(g_nvs_handle, KEY_VOLUME, state->volume);
    ret |= nvs_set_u8(g_nvs_handle, KEY_MAX_VOL, state->max_volume);
    ret |= nvs_set_str(g_nvs_handle, KEY_SSID, state->last_ssid);
    ret |= nvs_set_u32(g_nvs_handle, KEY_SIZE, state->download_size);
    ret |= nvs_set_u8(g_nvs_handle, KEY_PROGRESS, state->download_progress);
    
    if (ret == ESP_OK) {
        ret = nvs_commit(g_nvs_handle);
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save state: %s", esp_err_to_name(ret));
    } else {
        memcpy(&g_current_state, state, sizeof(boot_state_t));
        ESP_LOGI(TAG, "State saved: reason=%d, folder=%s, retry=%d, progress=%d%%",
                state->reason, state->folder, state->retry_count, state->download_progress);
    }
    
    return ret;
}

esp_err_t boot_mgr_load_state(boot_state_t* state) {
    if (!state || g_nvs_handle == 0) return ESP_ERR_INVALID_STATE;
    
    memset(state, 0, sizeof(boot_state_t));
    state->reason = BOOT_REASON_NORMAL;
    state->volume = 6;
    state->max_volume = 12;
    
    uint8_t temp;
    
    if (nvs_get_u8(g_nvs_handle, KEY_REASON, &temp) == ESP_OK) {
        state->reason = (boot_reason_t)temp;
    }
    
    size_t str_len = sizeof(state->folder);
    nvs_get_str(g_nvs_handle, KEY_FOLDER, state->folder, &str_len);
    
    nvs_get_u8(g_nvs_handle, KEY_RETRY, &state->retry_count);
    
    if (nvs_get_u8(g_nvs_handle, KEY_ERROR, &temp) == ESP_OK) {
        state->last_error = (download_error_t)temp;
    }
    
    nvs_get_u8(g_nvs_handle, KEY_VOLUME, &state->volume);
    nvs_get_u8(g_nvs_handle, KEY_MAX_VOL, &state->max_volume);
    
    str_len = sizeof(state->last_ssid);
    nvs_get_str(g_nvs_handle, KEY_SSID, state->last_ssid, &str_len);
    
    nvs_get_u32(g_nvs_handle, KEY_SIZE, &state->download_size);
    nvs_get_u8(g_nvs_handle, KEY_PROGRESS, &state->download_progress);
    
    memcpy(&g_current_state, state, sizeof(boot_state_t));
    
    ESP_LOGI(TAG, "State loaded: reason=%d, folder=%s, retry=%d, progress=%d%%",
            state->reason, state->folder, state->retry_count, state->download_progress);
    
    return ESP_OK;
}

esp_err_t boot_mgr_clear_state(void) {
    boot_state_t clean_state = {
        .reason = BOOT_REASON_NORMAL,
        .folder = "",
        .retry_count = 0,
        .last_error = DOWNLOAD_ERROR_NONE,
        .volume = 6,
        .max_volume = 12,
        .last_ssid = "",
        .download_size = 0,
        .download_progress = 0
    };
    
    return boot_mgr_save_state(&clean_state);
}

esp_err_t boot_mgr_request_download(const char* folder, uint32_t expected_size) {
    if (!folder) return ESP_ERR_INVALID_ARG;
    
    boot_state_t state = g_current_state;
    
    state.reason = BOOT_REASON_DOWNLOAD_REQUEST;
    strncpy(state.folder, folder, sizeof(state.folder) - 1);
    state.folder[sizeof(state.folder) - 1] = '\0';
    state.retry_count = 0;
    state.last_error = DOWNLOAD_ERROR_NONE;
    state.download_size = expected_size;
    state.download_progress = 0;
    
    ESP_LOGI(TAG, "Requesting download: %s (%lu bytes)", folder, (unsigned long)expected_size);    
    return boot_mgr_save_state(&state);
}

esp_err_t boot_mgr_update_progress(uint8_t percent) {
    if (g_nvs_handle == 0) return ESP_ERR_INVALID_STATE;
    
    g_current_state.download_progress = percent;
    
    esp_err_t ret = nvs_set_u8(g_nvs_handle, KEY_PROGRESS, percent);
    if (ret == ESP_OK) {
        ret = nvs_commit(g_nvs_handle);
    }
    
    return ret;
}

esp_err_t boot_mgr_report_success(void) {
    g_current_state.reason = BOOT_REASON_DOWNLOAD_SUCCESS;
    g_current_state.last_error = DOWNLOAD_ERROR_NONE;
    g_current_state.download_progress = 100;
    
    ESP_LOGI(TAG, "Download success reported");
    return boot_mgr_save_state(&g_current_state);
}

esp_err_t boot_mgr_report_failure(download_error_t error) {
    g_current_state.reason = BOOT_REASON_DOWNLOAD_FAILED;
    g_current_state.last_error = error;
    g_current_state.retry_count++;
    
    ESP_LOGI(TAG, "Download failure reported: %s (retry %d)",
            boot_mgr_get_error_string(error), g_current_state.retry_count);
    
    return boot_mgr_save_state(&g_current_state);
}

bool boot_mgr_needs_download(void) {
    return (g_current_state.reason == BOOT_REASON_DOWNLOAD_REQUEST) && 
           (strlen(g_current_state.folder) > 0);
}

esp_err_t boot_mgr_set_volume(uint8_t vol) {
    if (g_nvs_handle == 0) return ESP_ERR_INVALID_STATE;
    
    g_current_state.volume = vol;
    
    esp_err_t ret = nvs_set_u8(g_nvs_handle, KEY_VOLUME, vol);
    if (ret == ESP_OK) {
        ret = nvs_commit(g_nvs_handle);
    }
    
    return ret;
}

esp_err_t boot_mgr_get_volume(uint8_t* vol) {
    if (!vol) return ESP_ERR_INVALID_ARG;
    
    *vol = g_current_state.volume;
    return ESP_OK;
}

const char* boot_mgr_get_error_string(download_error_t error) {
    switch (error) {
        case DOWNLOAD_ERROR_NONE: return "No error";
        case DOWNLOAD_ERROR_WIFI_CONNECT: return "WiFi connection failed";
        case DOWNLOAD_ERROR_SERVER_UNREACHABLE: return "Server unreachable";
        case DOWNLOAD_ERROR_DOWNLOAD_FAILED: return "Download failed";
        case DOWNLOAD_ERROR_VERIFICATION_FAILED: return "Verification failed";
        case DOWNLOAD_ERROR_SD_CARD: return "SD card error";
        case DOWNLOAD_ERROR_TIMEOUT: return "Operation timeout";
        default: return "Unknown error";
    }
}