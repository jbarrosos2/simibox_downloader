/* downloader_main.cpp - ESP-IDF Downloader (C++ main) with Debug Mode */
#include "simibox_download.h"
#include "simibox_boot_manager_idf.h"
#include "simibox_wifi_creds_idf.h"
#include "simibox_led_idf.h"
#include "downloader_config.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <string>

static const char* TAG = "downloader";
static const char* LAMBDA_URL = "https://oqgyreq4gabsrqvfu4i3lxumse0mrcdb.lambda-url.us-east-2.on.aws/";
static const char* MAIN_LABEL = "musicbox";
static const esp_partition_subtype_t MAIN_SUB = ESP_PARTITION_SUBTYPE_APP_OTA_0;

static wifi_creds_handle_t wifi_handle;
static boot_state_t boot_state;

#if SEED_WIFI_CREDS
static void seed_wifi_credentials(void) {
    ESP_LOGW(TAG, "DEBUG: Seeding WiFi credentials for standalone testing");
    
    nvs_handle_t nvs;
    esp_err_t ret = nvs_open("wifi_creds", NVS_READWRITE, &nvs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for WiFi creds: %s", esp_err_to_name(ret));
        return;
    }
    
    // Clear any existing WiFi credentials
    nvs_erase_all(nvs);
    
    // Set one network
    nvs_set_u8(nvs, "count", 1);
    nvs_set_str(nvs, "ssid_0", TEST_WIFI_SSID);
    nvs_set_str(nvs, "pass_0", TEST_WIFI_PASS);
    
    nvs_commit(nvs);
    nvs_close(nvs);
    
    ESP_LOGI(TAG, "DEBUG: Seeded WiFi network: %s", TEST_WIFI_SSID);
}

static bool connect_to_wifi_debug_mode(void) {
    ESP_LOGW(TAG, "DEBUG: Direct WiFi connection mode");
    led_show_wifi_connecting();
    
    // Initialize WiFi system
    ESP_ERROR_CHECK(wifi_creds_init(&wifi_handle));
    
    // Direct connection without scan in debug mode
    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.sta.ssid, TEST_WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, TEST_WIFI_PASS, sizeof(wifi_config.sta.password));
    
    ESP_LOGI(TAG, "DEBUG: Connecting directly to %s...", TEST_WIFI_SSID);
    
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // Wait for connection (simpler for debug mode)
    for (int i = 0; i < 300; i++) {  // 30 seconds max
        led_update();
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Check if connected
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            ESP_LOGI(TAG, "DEBUG: Connected to %s!", TEST_WIFI_SSID);
            strncpy(boot_state.last_ssid, TEST_WIFI_SSID, sizeof(boot_state.last_ssid) - 1);
            led_set_color(0, LEDC_MAX_DUTY, 0);
            vTaskDelay(pdMS_TO_TICKS(1000));
            return true;
        }
    }
    
    ESP_LOGE(TAG, "DEBUG: Failed to connect to %s", TEST_WIFI_SSID);
    led_show_error();
    return false;
}
#endif

static const esp_partition_t* find_musicbox_partition(void) {
    const esp_partition_t* p = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, MAIN_LABEL);
    if (!p) {
        p = esp_partition_find_first(ESP_PARTITION_TYPE_APP, MAIN_SUB, nullptr);
    }
    return p;
}

static void boot_musicbox_and_restart(void) {
#if STAY_IN_DOWNLOADER
    ESP_LOGW(TAG, "DEBUG: STAY_IN_DOWNLOADER mode - not booting musicbox");
    ESP_LOGW(TAG, "DEBUG: Restarting downloader for another test...");
    esp_restart();
#else
    const esp_partition_t* mainp = find_musicbox_partition();
    if (!mainp) {
        ESP_LOGE(TAG, "musicbox partition not found; restarting anyway");
        esp_restart();
    }
    
    ESP_ERROR_CHECK(esp_ota_set_boot_partition(mainp));
    ESP_LOGI(TAG, "Switching to musicbox firmware @ 0x%08lx", (unsigned long)mainp->address);
    esp_restart();
#endif
}

static bool connect_to_wifi_with_feedback(void) {
#if SKIP_WIFI_SCAN && SEED_WIFI_CREDS
    return connect_to_wifi_debug_mode();
#else
    ESP_LOGI(TAG, "Connecting to WiFi using saved credentials...");
    
    led_show_wifi_connecting();
    
    for (int i = 0; i < 50; i++) {
        led_update();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    
    if (wifi_creds_load(&wifi_handle) != ESP_OK) {
        ESP_LOGE(TAG, "No saved WiFi networks found!");
        led_show_error();
        return false;
    }
    
    if (wifi_creds_connect_best(&wifi_handle, 30000) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect to any saved network");
        led_show_error();
        return false;
    }
    
    char ssid[32];
    wifi_creds_get_current_ssid(ssid, sizeof(ssid));
    ESP_LOGI(TAG, "Connected to: %s", ssid);
    
    strncpy(boot_state.last_ssid, ssid, sizeof(boot_state.last_ssid) - 1);
    
    led_set_color(0, LEDC_MAX_DUTY, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    return true;
#endif
}

static bool handle_download_with_retry(void) {
    ESP_LOGI(TAG, "Starting download of folder: %s", boot_state.folder);
    ESP_LOGI(TAG, "Retry attempt: %d", boot_state.retry_count + 1);
    
    led_show_downloading();
    
    ESP_LOGI(TAG, "Mounting SD card...");
    simibox_download::mount_sd("/sdcard");
    
    // Use your existing C++ download function directly!
    bool success = simibox_download::download_simi_folder(
        std::string(boot_state.folder), 
        std::string(LAMBDA_URL)
    );
    
    simibox_download::unmount_sd("/sdcard");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    if (success) {
        ESP_LOGI(TAG, "Download completed successfully!");
        
        led_show_success();
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        for (int i = 0; i < 30; i++) {
            uint32_t hue = (i * 12) % 360;
            uint16_t r = (hue < 120) ? LEDC_MAX_DUTY : 0;
            uint16_t g = (hue >= 120 && hue < 240) ? LEDC_MAX_DUTY : 0;
            uint16_t b = (hue >= 240) ? LEDC_MAX_DUTY : 0;
            led_set_color(r, g, b);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        return true;
    } else {
        ESP_LOGE(TAG, "Download failed!");
        
        led_show_error();
        for (int i = 0; i < 10; i++) {
            led_update();
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        
        return false;
    }
}

extern "C" void app_main(void) {
    // Set log level for debug mode
    esp_log_level_set("*", DEFAULT_LOG_LEVEL);
    
#if DOWNLOADER_DEV_MODE
    ESP_LOGW(TAG, "==========================================");
    ESP_LOGW(TAG, "     DEBUG MODE ENABLED");
    ESP_LOGW(TAG, "     Test WiFi: %s", TEST_WIFI_SSID);
    ESP_LOGW(TAG, "     Test Folder: %s", TEST_FOLDER);
    ESP_LOGW(TAG, "==========================================");
#endif
    
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    
#if SEED_WIFI_CREDS
    seed_wifi_credentials();
#endif
    
    led_init();
    led_set_color(LEDC_MAX_DUTY, 0, LEDC_MAX_DUTY);  // Purple = booting
    
    ESP_ERROR_CHECK(boot_mgr_init());
    ESP_ERROR_CHECK(boot_mgr_load_state(&boot_state));
    
    const esp_partition_t* running = esp_ota_get_running_partition();
    ESP_LOGI(TAG, "Running from partition: %s @ 0x%08lx", 
            running->label, (unsigned long)running->address);
    
#if FORCE_DOWNLOAD
    ESP_LOGW(TAG, "DEBUG: FORCE_DOWNLOAD mode - seeding download request");
    boot_mgr_request_download(TEST_FOLDER, 0);
    boot_mgr_load_state(&boot_state);
#endif
    
    if (!boot_mgr_needs_download()) {
        ESP_LOGI(TAG, "No download requested, returning to musicbox");
        boot_musicbox_and_restart();
    }
    
    ESP_LOGI(TAG, "Download requested for: %s", boot_state.folder);
    
    if (boot_state.retry_count >= 3) {
        ESP_LOGE(TAG, "Max retries exceeded, giving up");
        boot_mgr_report_failure(DOWNLOAD_ERROR_DOWNLOAD_FAILED);
        led_show_error();
        vTaskDelay(pdMS_TO_TICKS(3000));
        boot_musicbox_and_restart();
    }
    
#if !SKIP_WIFI_SCAN
    ESP_ERROR_CHECK(wifi_creds_init(&wifi_handle));
#endif
    
    if (!connect_to_wifi_with_feedback()) {
        boot_mgr_report_failure(DOWNLOAD_ERROR_WIFI_CONNECT);
        vTaskDelay(pdMS_TO_TICKS(3000));
        boot_musicbox_and_restart();
    }
    
    bool success = handle_download_with_retry();
    
    if (success) {
        boot_mgr_report_success();
        ESP_LOGI(TAG, "=== DOWNLOAD SUCCESS ===");
        
#if STAY_IN_DOWNLOADER
        ESP_LOGW(TAG, "DEBUG: STAY_IN_DOWNLOADER mode - restarting downloader");
        esp_restart();
#else
        boot_musicbox_and_restart();
#endif
    } else {
        boot_mgr_report_failure(DOWNLOAD_ERROR_DOWNLOAD_FAILED);
        vTaskDelay(pdMS_TO_TICKS(2000));
        boot_musicbox_and_restart();
    }
}