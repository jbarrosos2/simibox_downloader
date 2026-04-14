/* downloader_main.cpp - ESP-IDF Downloader (C++ main) with Debug Mode
 * 
 * FIXED: Uses wifi_creds_connect_auto() which leverages ESP32's auto-connect
 * feature. The ESP32 remembers the last connected network from musicbox
 * and auto-reconnects on boot.
 * 
 * HW v2: No SD power switch (GPIO13 is now MAX98357A SD_MODE#).
 * SD card recovery relies on SPI bus reset, not power cycling.
 */
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
    ESP_ERROR_CHECK(esp_wifi_connect());
    
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
    ESP_LOGI(TAG, "Connecting to WiFi...");
    
    led_show_wifi_connecting();
    
    // Show connecting animation briefly
    for (int i = 0; i < 20; i++) {
        led_update();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    // Initialize WiFi subsystem
    if (wifi_creds_init(&wifi_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize WiFi");
        led_show_error();
        return false;
    }
    
    // ═══════════════════════════════════════════════════════════════════════════
    // FIX: Use wifi_creds_connect_auto() instead of wifi_creds_connect_best()
    // 
    // This function:
    // 1. First waits for ESP32's auto-connect (it remembers from musicbox!)
    // 2. If auto-connect fails, falls back to our saved networks
    // 
    // The old code failed because it checked OUR NVS namespace for saved
    // networks, but the ESP32 driver already auto-connected using ITS
    // internal storage. By the time we declared "no networks found",
    // the ESP32 was already connected!
    // ═══════════════════════════════════════════════════════════════════════════
    
    ESP_LOGI(TAG, "Using auto-connect strategy (30s timeout)...");
    
    // Keep updating LED while waiting
    esp_err_t ret = ESP_FAIL;
    
    // Try auto-connect with LED feedback
    for (int attempt = 0; attempt < 3 && ret != ESP_OK; attempt++) {
        if (attempt > 0) {
            ESP_LOGW(TAG, "Retry attempt %d...", attempt + 1);
        }
        
        // Update LED animation during connection
        for (int i = 0; i < 100; i++) {  // 10 seconds per attempt
            led_update();
            vTaskDelay(pdMS_TO_TICKS(100));
            
            // Check if connected
            if (wifi_creds_is_connected()) {
                ret = ESP_OK;
                break;
            }
        }
        
        // Trigger connection if not connected yet
        if (ret != ESP_OK && attempt == 0) {
            ret = wifi_creds_wait_for_connection(10000);  // 10 seconds
        }
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect to WiFi");
        led_show_error();
        return false;
    }
    
    // Get and log connection info
    char ssid[32] = {0};
    wifi_creds_get_current_ssid(ssid, sizeof(ssid));
    ESP_LOGI(TAG, "Connected to: %s", ssid);
    
    int8_t rssi;
    uint8_t channel;
    if (wifi_creds_get_connection_info(&rssi, &channel) == ESP_OK) {
        ESP_LOGI(TAG, "Signal: %d dBm, Channel: %d", rssi, channel);
    }
    
    strncpy(boot_state.last_ssid, ssid, sizeof(boot_state.last_ssid) - 1);
    
    // Brief green flash to confirm connection
    led_set_color(0, LEDC_MAX_DUTY, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    return true;
#endif
}

static bool handle_download_with_retry(void) {
    ESP_LOGI(TAG, "Starting download of folder: %s", boot_state.folder);
    ESP_LOGI(TAG, "Retry attempt: %d", boot_state.retry_count + 1);
    
    led_show_downloading();
    
    // CRITICAL: Clean up any stale SPI/SD state from previous crash
    // HW v2: uses SPI bus reset only (no power cycle available)
    simibox_download::force_clean_sd_bus();
    
    ESP_LOGI(TAG, "Mounting SD card...");
    if (!simibox_download::mount_sd("/sdcard")) {
        ESP_LOGE(TAG, "SD card mount failed - cannot proceed with download");
        led_show_error();
        vTaskDelay(pdMS_TO_TICKS(2000));
        // Unmount with card_healthy=false to skip filesystem sync
        simibox_download::unmount_sd("/sdcard", false);
        // Report SD card error - counts as a retry attempt
        boot_mgr_report_failure(DOWNLOAD_ERROR_SD_CARD);
        return false;
    }
    
    // Download with SD health tracking
    bool sd_failed = false;
    bool success = simibox_download::download_simi_folder(
        std::string(boot_state.folder), 
        std::string(LAMBDA_URL),
        &sd_failed
    );
    
    // ═══════════════════════════════════════════════════════════════════════
    // Unmount based on actual failure type:
    //   SD failure  → card_healthy=false (skip sync writes)
    //   Net failure → card_healthy=true  (normal unmount, card is fine)
    //   Success     → card_healthy=true
    // ═══════════════════════════════════════════════════════════════════════
    simibox_download::unmount_sd("/sdcard", !sd_failed);
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
        ESP_LOGE(TAG, "Download failed! (SD issue: %s)", sd_failed ? "YES" : "no");
        
        led_show_error();
        for (int i = 0; i < 10; i++) {
            led_update();
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        
        // Report failure with appropriate error code
        boot_mgr_report_failure(sd_failed ? DOWNLOAD_ERROR_SD_CARD 
                                          : DOWNLOAD_ERROR_DOWNLOAD_FAILED);
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
    
    // ════════════════════════════════════════════════════════════════════════════
    // NVS INIT - SELECTIVE ERASE TO PRESERVE HMAC KEY
    // 
    // CRITICAL: We must NOT use nvs_flash_erase() as it would destroy the HMAC
    // secret key stored in the "simibox" namespace, effectively bricking the
    // device (all tags would be rejected forever with no user recovery path).
    //
    // Instead, we only erase namespaces we control:
    //   - bootmgr: boot protocol state (safe to erase)
    //   - wifi_creds: WiFi credentials (can be re-provisioned)
    //   - known_nets: WiFi credentials from musicbox (can be re-provisioned)
    //
    // We PRESERVE:
    //   - simibox: contains HMAC secret key (MUST NOT ERASE)
    // ════════════════════════════════════════════════════════════════════════════
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS needs recovery (error: %s)", esp_err_to_name(err));
        ESP_LOGW(TAG, "Performing SELECTIVE erase to preserve HMAC key...");
        
        // First, try to erase just our safe-to-erase namespaces
        // This won't help with NO_FREE_PAGES but is worth trying first
        nvs_handle_t h;
        
        // Erase bootmgr namespace (boot protocol state)
        if (nvs_open("bootmgr", NVS_READWRITE, &h) == ESP_OK) {
            nvs_erase_all(h);
            nvs_commit(h);
            nvs_close(h);
            ESP_LOGI(TAG, "Erased 'bootmgr' namespace");
        }
        
        // Erase wifi_creds namespace (our WiFi storage)
        if (nvs_open("wifi_creds", NVS_READWRITE, &h) == ESP_OK) {
            nvs_erase_all(h);
            nvs_commit(h);
            nvs_close(h);
            ESP_LOGI(TAG, "Erased 'wifi_creds' namespace");
        }
        
        // Erase known_nets namespace (musicbox WiFi storage)
        if (nvs_open("known_nets", NVS_READWRITE, &h) == ESP_OK) {
            nvs_erase_all(h);
            nvs_commit(h);
            nvs_close(h);
            ESP_LOGI(TAG, "Erased 'known_nets' namespace");
        }
        
        // Try init again
        err = nvs_flash_init();
        
        if (err != ESP_OK) {
            // If selective erase didn't help, we have no choice but full erase
            // This will lose the HMAC key - log a critical warning
            ESP_LOGE(TAG, "═══════════════════════════════════════════════════════════");
            ESP_LOGE(TAG, "CRITICAL: NVS still corrupt, must perform FULL erase!");
            ESP_LOGE(TAG, "WARNING: This will DESTROY the HMAC secret key!");
            ESP_LOGE(TAG, "Device will need to be re-provisioned after this!");
            ESP_LOGE(TAG, "═══════════════════════════════════════════════════════════");
            
            ESP_ERROR_CHECK(nvs_flash_erase());
            ESP_ERROR_CHECK(nvs_flash_init());
        }
    }
    
#if SEED_WIFI_CREDS
    seed_wifi_credentials();
#endif
    
    // ════════════════════════════════════════════════════════════════════════════
    // BOOT COUNT - Anti-brick mechanism for downloader
    // Same logic as musicbox: track consecutive crashes and enter safe mode
    // if we keep failing before completing the download task.
    // ════════════════════════════════════════════════════════════════════════════
    {
        nvs_handle_t boot_nvs;
        if (nvs_open(SIMIBOX_NVS_NAMESPACE, NVS_READWRITE, &boot_nvs) == ESP_OK) {
            uint8_t boot_count = 0;
            nvs_get_u8(boot_nvs, NVS_KEY_BOOT_COUNT, &boot_count);
            boot_count++;
            nvs_set_u8(boot_nvs, NVS_KEY_BOOT_COUNT, boot_count);
            nvs_commit(boot_nvs);
            nvs_close(boot_nvs);
            
            ESP_LOGI(TAG, "Boot count: %d/%d", boot_count, MAX_BOOT_FAILURES);
            
            if (boot_count >= MAX_BOOT_FAILURES) {
                ESP_LOGE(TAG, "═══════════════════════════════════════════════════════════");
                ESP_LOGE(TAG, "CRITICAL: MAX BOOT FAILURES EXCEEDED IN DOWNLOADER!");
                ESP_LOGE(TAG, "Downloader may be stuck in a crash loop.");
                ESP_LOGE(TAG, "Resetting boot count and returning to musicbox.");
                ESP_LOGE(TAG, "═══════════════════════════════════════════════════════════");
                
                // Reset boot count
                if (nvs_open(SIMIBOX_NVS_NAMESPACE, NVS_READWRITE, &boot_nvs) == ESP_OK) {
                    nvs_set_u8(boot_nvs, NVS_KEY_BOOT_COUNT, 0);
                    nvs_commit(boot_nvs);
                    nvs_close(boot_nvs);
                }
                
                // Show error briefly then return to musicbox
                led_init();
                led_show_error();
                for (int i = 0; i < 30; i++) {  // 3 seconds of error
                    led_update();
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                
                boot_musicbox_and_restart();
            }
        }
    }
    
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
    
    // Connect to WiFi (uses auto-connect strategy)
    if (!connect_to_wifi_with_feedback()) {
        boot_mgr_report_failure(DOWNLOAD_ERROR_WIFI_CONNECT);
        vTaskDelay(pdMS_TO_TICKS(3000));
        boot_musicbox_and_restart();
    }
    
    bool success = handle_download_with_retry();
    
    if (success) {
        boot_mgr_report_success();
        ESP_LOGI(TAG, "=== DOWNLOAD SUCCESS ===");
        
        // Reset boot count on successful download
        nvs_handle_t boot_nvs;
        if (nvs_open(SIMIBOX_NVS_NAMESPACE, NVS_READWRITE, &boot_nvs) == ESP_OK) {
            nvs_set_u8(boot_nvs, NVS_KEY_BOOT_COUNT, 0);
            nvs_commit(boot_nvs);
            nvs_close(boot_nvs);
            ESP_LOGI(TAG, "Boot count reset - download successful");
        }
        
#if STAY_IN_DOWNLOADER
        // Debug mode: show success and halt (no reboot)
        led_show_success();
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "╔════════════════════════════════════════╗");
        ESP_LOGI(TAG, "║   ¡Descarga completa con éxito!        ║");
        ESP_LOGI(TAG, "║   Carpeta: %-28s║", boot_state.folder);
        ESP_LOGI(TAG, "╚════════════════════════════════════════╝");
        ESP_LOGI(TAG, "");
        ESP_LOGW(TAG, "DEBUG: STAY_IN_DOWNLOADER - sistema detenido");
        ESP_LOGW(TAG, "       Presiona RESET para reiniciar");
        
        // Idle forever with green LED
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
#else
        boot_musicbox_and_restart();
#endif
    } else {
        // Failure already reported inside handle_download_with_retry()
        // Just wait and restart
        vTaskDelay(pdMS_TO_TICKS(2000));
        boot_musicbox_and_restart();
    }
}