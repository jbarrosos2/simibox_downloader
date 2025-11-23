#include "simibox_download.h"
#include "downloader_config.h"


#include "esp_log.h"
#include "esp_system.h"      // esp_restart()
#include "esp_ota_ops.h"     // esp_ota_set_boot_partition(), esp_ota_get_running_partition()
#include "esp_partition.h"   // esp_partition_find_first()
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>        // PRIx32
#include <stdlib.h>          // malloc, free

// ---------- Config ----------
static const char* TAG                  = "app_main";

// NVS contract shared with musicbox firmware
static constexpr const char* NVS_NS     = "bootmgr";
static constexpr const char* KEY_DL     = "download";   // u8 (0/1)
static constexpr const char* KEY_FOLDER = "folder";     // str

// Partition labels/subtypes (must match your partitions_dualapp.csv)
static constexpr const char* MAIN_LABEL = "musicbox";
static constexpr esp_partition_subtype_t MAIN_SUB = ESP_PARTITION_SUBTYPE_APP_OTA_0;

// Network / service
static constexpr const char* WIFI_SSID  = "Parcela 41";
static constexpr const char* WIFI_PASS  = "parcela41wifi";
static constexpr const char* LAMBDA_URL = "https://oqgyreq4gabsrqvfu4i3lxumse0mrcdb.lambda-url.us-east-2.on.aws/";

// ---------- Helpers ----------
static const esp_partition_t* find_musicbox_partition() {
    // Prefer label from CSV; fall back to subtype
    const esp_partition_t* p = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, MAIN_LABEL);
    if (!p) {
        p = esp_partition_find_first(
            ESP_PARTITION_TYPE_APP, MAIN_SUB, nullptr);
    }
    return p;
}

static void boot_musicbox_and_restart() {
    const esp_partition_t* mainp = find_musicbox_partition();
    if (!mainp) {
        ESP_LOGE(TAG, "musicbox partition not found; restarting without changing boot slot");
        esp_restart(); // fallback to current boot slot
    }
    ESP_ERROR_CHECK(esp_ota_set_boot_partition(mainp));
    ESP_LOGI(TAG, "Set boot to '%s' @ 0x%08" PRIx32 ", restarting...",
             mainp->label, (uint32_t)mainp->address);
    esp_restart();
}

extern "C" void app_main() {
    // 1) Init NVS (handle full/old pages)
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // 2) Open namespace READWRITE (creates on fresh board)
    nvs_handle_t h = 0;
    err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    #if FORCE_DOWNLOAD
        ESP_LOGW(TAG, "FORCE_DOWNLOAD: seeding NVS download=1, folder=\"" TEST_FOLDER "\"");
        ESP_ERROR_CHECK(nvs_set_u8(h, KEY_DL, 1));
        ESP_ERROR_CHECK(nvs_set_str(h, KEY_FOLDER, TEST_FOLDER));
        ESP_ERROR_CHECK(nvs_commit(h));
    #endif
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open(%s) failed: %s", NVS_NS, esp_err_to_name(err));
        boot_musicbox_and_restart();
    }

    // 3) Read one-shot flag
    uint8_t download = 0;
    err = nvs_get_u8(h, KEY_DL, &download);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "'%s' not found; seeding default 0", KEY_DL);
        ESP_ERROR_CHECK(nvs_set_u8(h, KEY_DL, 0));
        ESP_ERROR_CHECK(nvs_commit(h));
        download = 0;
    } else {
        ESP_ERROR_CHECK(err);
    }

    // 4) Read folder string
    size_t folder_len = 0;
    char* folder = nullptr;
    err = nvs_get_str(h, KEY_FOLDER, nullptr, &folder_len);
    if (err == ESP_OK && folder_len > 1) { // >1 to include '\0'
        folder = (char*)malloc(folder_len);
        if (!folder) {
            ESP_LOGE(TAG, "malloc(%u) failed", (unsigned)folder_len);
            nvs_close(h);
            boot_musicbox_and_restart();
        }
        ESP_ERROR_CHECK(nvs_get_str(h, KEY_FOLDER, folder, &folder_len));
        ESP_LOGI(TAG, "Requested folder: '%s'", folder);
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "'%s' not found", KEY_FOLDER);
    } else if (err != ESP_OK) {
        ESP_ERROR_CHECK(err);
    }

    // 5) If no request, bounce back immediately
    if (!download || !folder || folder[0] == '\0') {
        ESP_LOGI(TAG, "No download requested (download=%u, folder=%p). Booting musicbox.",
                 download, (void*)folder);
        if (folder) free(folder);
        nvs_close(h);
        boot_musicbox_and_restart();
    }

    // 6) Do the work
    ESP_LOGI(TAG, "Initializing WiFi + SD and downloading '%s'...", folder);
    simibox_download::init_wifi(WIFI_SSID, WIFI_PASS);
    simibox_download::mount_sd("/sdcard");

    bool ok = simibox_download::download_simi_folder(folder, LAMBDA_URL);
    ESP_LOGI(TAG, "download_simi_folder returned %s", ok ? "OK" : "FAIL");

    // IMPORTANT: Unmount SD card before reboot to ensure all data is written
    simibox_download::unmount_sd("/sdcard");
    vTaskDelay(pdMS_TO_TICKS(1000));  // Small delay to ensure controller completes

    // 7) On success: clear one-shot flag, commit, and boot back to musicbox
    if (ok) {
    #if defined(STAY_IN_DOWNLOADER) && STAY_IN_DOWNLOADER
        ESP_LOGW(TAG, "STAY_IN_DOWNLOADER: keeping download=1 and restarting downloader");
        // Leave NVS as-is and just restart current app
        if (folder) free(folder);
        nvs_close(h);
        esp_restart();
    #else
        ESP_ERROR_CHECK(nvs_set_u8(h, KEY_DL, 0));
        ESP_ERROR_CHECK(nvs_commit(h));
        if (folder) free(folder);
        nvs_close(h);
        boot_musicbox_and_restart();
    #endif
    }
    
    // Also unmount on failure path
    simibox_download::unmount_sd("/sdcard");
    vTaskDelay(pdMS_TO_TICKS(200));
    
    if (folder) free(folder);
    nvs_close(h);
    boot_musicbox_and_restart();
}