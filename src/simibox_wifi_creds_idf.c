/* simibox_wifi_creds_idf.c - Pure ESP-IDF WiFi Credential Implementation */
#include "simibox_wifi_creds_idf.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h" 
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include <string.h>

static const char* TAG = "wifi_creds";
static const char* NVS_NAMESPACE = "wifi_creds";
static const char* NVS_KEY_COUNT = "count";
static const char* NVS_KEY_SSID_PREFIX = "ssid_";
static const char* NVS_KEY_PASS_PREFIX = "pass_";

static EventGroupHandle_t wifi_event_group;
static const int CONNECTED_BIT = BIT0;

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        ESP_LOGI(TAG, "Retry connecting to AP");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
    }
}

esp_err_t wifi_creds_init(wifi_creds_handle_t* handle) {
    if (!handle) return ESP_ERR_INVALID_ARG;
    
    memset(handle, 0, sizeof(wifi_creds_handle_t));
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    wifi_event_group = xEventGroupCreate();
    
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                              &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                              &wifi_event_handler, NULL));
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    handle->initialized = true;
    ESP_LOGI(TAG, "WiFi credentials manager initialized");
    
    return ESP_OK;
}

esp_err_t wifi_creds_load(wifi_creds_handle_t* handle) {
    if (!handle || !handle->initialized) return ESP_ERR_INVALID_STATE;
    
    nvs_handle_t nvs;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "No saved networks found");
        return ESP_OK;
    }
    
    uint8_t count = 0;
    ret = nvs_get_u8(nvs, NVS_KEY_COUNT, &count);
    if (ret != ESP_OK || count == 0) {
        nvs_close(nvs);
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Loading %d saved networks", count);
    handle->count = 0;
    
    for (uint8_t i = 0; i < count && i < WIFI_MAX_NETWORKS; i++) {
        char key[32];
        size_t ssid_len = WIFI_SSID_MAX_LEN;
        size_t pass_len = WIFI_PASS_MAX_LEN;
        
        snprintf(key, sizeof(key), "%s%d", NVS_KEY_SSID_PREFIX, i);
        ret = nvs_get_str(nvs, key, handle->networks[handle->count].ssid, &ssid_len);
        if (ret != ESP_OK) continue;
        
        snprintf(key, sizeof(key), "%s%d", NVS_KEY_PASS_PREFIX, i);
        ret = nvs_get_str(nvs, key, handle->networks[handle->count].password, &pass_len);
        if (ret != ESP_OK) continue;
        
        handle->networks[handle->count].rssi = -100;
        handle->count++;
        
        ESP_LOGI(TAG, "  [%d] %s", i, handle->networks[i].ssid);
    }
    
    nvs_close(nvs);
    ESP_LOGI(TAG, "Loaded %d networks", handle->count);
    
    return ESP_OK;
}

esp_err_t wifi_creds_scan_and_match(wifi_creds_handle_t* handle) {
    if (!handle || !handle->initialized) return ESP_ERR_INVALID_STATE;
    
    for (int i = 0; i < handle->count; i++) {
        handle->networks[i].rssi = -100;
    }
    
    wifi_scan_config_t scan_cfg = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 100,
        .scan_time.active.max = 1500,
    };
    
    ESP_LOGI(TAG, "Starting WiFi scan...");
    ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_cfg, true));
    
    uint16_t ap_count = WIFI_SCAN_MAX_AP;
    wifi_ap_record_t ap_list[WIFI_SCAN_MAX_AP];
    
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_count, ap_list));
    ESP_LOGI(TAG, "Found %d access points", ap_count);
    
    for (int i = 0; i < ap_count; i++) {
        for (int j = 0; j < handle->count; j++) {
            if (strcmp((char*)ap_list[i].ssid, handle->networks[j].ssid) == 0) {
                handle->networks[j].rssi = ap_list[i].rssi;
                ESP_LOGI(TAG, "  [Match] %s (RSSI: %d dBm)", 
                        handle->networks[j].ssid, handle->networks[j].rssi);
            }
        }
    }
    
    for (int i = 0; i < handle->count - 1; i++) {
        for (int j = 0; j < handle->count - i - 1; j++) {
            if (handle->networks[j].rssi < handle->networks[j + 1].rssi) {
                wifi_network_t temp = handle->networks[j];
                handle->networks[j] = handle->networks[j + 1];
                handle->networks[j + 1] = temp;
            }
        }
    }
    
    return ESP_OK;
}

esp_err_t wifi_creds_connect_best(wifi_creds_handle_t* handle, uint32_t timeout_ms) {
    if (!handle || !handle->initialized || handle->count == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_ERROR_CHECK(wifi_creds_scan_and_match(handle));
    
    for (int i = 0; i < handle->count; i++) {
        if (handle->networks[i].rssi <= -100) continue;
        
        ESP_LOGI(TAG, "Connecting to %s (RSSI: %d)...", 
                handle->networks[i].ssid, handle->networks[i].rssi);
        
        wifi_config_t wifi_config = {0};
        strncpy((char*)wifi_config.sta.ssid, handle->networks[i].ssid, 
                sizeof(wifi_config.sta.ssid));
        strncpy((char*)wifi_config.sta.password, handle->networks[i].password, 
                sizeof(wifi_config.sta.password));
        
        ESP_ERROR_CHECK(esp_wifi_stop());
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());
        
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                              CONNECTED_BIT,
                                              pdFALSE,
                                              pdFALSE,
                                              pdMS_TO_TICKS(timeout_ms / handle->count));
        
        if (bits & CONNECTED_BIT) {
            ESP_LOGI(TAG, "Connected to %s", handle->networks[i].ssid);
            return ESP_OK;
        }
        
        ESP_LOGW(TAG, "Failed to connect to %s", handle->networks[i].ssid);
    }
    
    ESP_LOGE(TAG, "Failed to connect to any saved network");
    return ESP_FAIL;
}

esp_err_t wifi_creds_get_current_ssid(char* ssid, size_t max_len) {
    if (!ssid || max_len == 0) return ESP_ERR_INVALID_ARG;
    
    wifi_ap_record_t ap_info;
    esp_err_t ret = esp_wifi_sta_get_ap_info(&ap_info);
    if (ret == ESP_OK) {
        strncpy(ssid, (char*)ap_info.ssid, max_len - 1);
        ssid[max_len - 1] = '\0';
    }
    
    return ret;
}