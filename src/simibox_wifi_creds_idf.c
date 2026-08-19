/* simibox_wifi_creds_idf.c - Pure ESP-IDF WiFi Credential Implementation
 * 
 * OPTIMIZATIONS FOR THROUGHPUT:
 * - Power save disabled (WIFI_PS_NONE)
 * - 40MHz bandwidth (WIFI_BW_HT40)
 * - 11b/g/n protocols enabled
 * - These settings are CRITICAL for download throughput!
 * 
 * AUTO-CONNECT FIX:
 * - ESP32 WiFi driver remembers last connected network internally
 * - When musicbox connects, ESP32 saves credentials in its own NVS
 * - On reboot, ESP32 auto-reconnects WITHOUT us asking
 * - We just need to WAIT for it, not fight it!
 */
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
static const int DISCONNECTED_BIT = BIT1;
static bool s_wifi_initialized = false;
static bool s_is_connecting = false;

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        // Don't auto-call esp_wifi_connect() here - let auto-connect work
        // or we'll call it explicitly when needed
        ESP_LOGI(TAG, "WiFi STA started");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(TAG, "WiFi connected to AP (waiting for IP...)");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*) event_data;
        ESP_LOGW(TAG, "WiFi disconnected, reason: %d", event->reason);
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        xEventGroupSetBits(wifi_event_group, DISCONNECTED_BIT);
        
        // Only retry if we're actively trying to connect
        if (s_is_connecting) {
            ESP_LOGI(TAG, "Retrying connection...");
            esp_wifi_connect();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        xEventGroupClearBits(wifi_event_group, DISCONNECTED_BIT);
        s_is_connecting = false;  // We're connected, stop retrying
    }
}

esp_err_t wifi_creds_init(wifi_creds_handle_t* handle) {
    if (!handle) return ESP_ERR_INVALID_ARG;
    
    memset(handle, 0, sizeof(wifi_creds_handle_t));
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // Do NOT blindly nvs_flash_erase() here. NVS recovery is owned by
        // app_main (downloader_main.c). app_main normally initializes NVS before
        // this runs, so reaching this branch is unexpected; log and continue.
        // (The HMAC secret now lives in the dedicated 'simikey' partition, outside
        //  NVS, so even a full NVS erase can never destroy it.)
        ESP_LOGW(TAG, "NVS not initialized here (%s); expected app_main to own it",
                 esp_err_to_name(ret));
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
    
    // ═══════════════════════════════════════════════════════════════════════════════
    // CRITICAL THROUGHPUT OPTIMIZATIONS
    // Without these, you lose 30-50% of potential throughput!
    // ═══════════════════════════════════════════════════════════════════════════════
    
    // 1. Disable power save - keeps radio active, reduces latency
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_LOGI(TAG, "WiFi power save DISABLED for max throughput");
    
    // 2. Enable 40MHz bandwidth (HT40) - doubles PHY rate potential
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40));
    ESP_LOGI(TAG, "WiFi bandwidth set to HT40 (40MHz)");
    
    // 3. Enable all 802.11 protocols including 11n for best rates
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, 
        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));
    ESP_LOGI(TAG, "WiFi protocols: 11b/g/n enabled");
    
    // ═══════════════════════════════════════════════════════════════════════════════
    
    ESP_ERROR_CHECK(esp_wifi_start());
    
    handle->initialized = true;
    s_wifi_initialized = true;
    ESP_LOGI(TAG, "WiFi credentials manager initialized (throughput-optimized)");
    
    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// NEW: Wait for auto-connect (ESP32 remembers last network from musicbox)
// ═══════════════════════════════════════════════════════════════════════════════
esp_err_t wifi_creds_wait_for_connection(uint32_t timeout_ms) {
    ESP_LOGI(TAG, "Waiting for WiFi connection (auto-connect or manual)...");
    
    // Check if already connected
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        ESP_LOGI(TAG, "Already connected to: %s", ap_info.ssid);
        return ESP_OK;
    }
    
    // Trigger connect (ESP32 will use remembered credentials)
    s_is_connecting = true;
    esp_wifi_connect();
    
    // Wait for IP
    ESP_LOGI(TAG, "Waiting up to %lu ms for IP...", (unsigned long)timeout_ms);
    
    EventBits_t bits = xEventGroupWaitBits(
        wifi_event_group,
        CONNECTED_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(timeout_ms)
    );
    
    s_is_connecting = false;
    
    if (bits & CONNECTED_BIT) {
        // Get connected network info
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            ESP_LOGI(TAG, "Connected to: %s (RSSI: %d)", ap_info.ssid, ap_info.rssi);
        }
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "Connection timeout after %lu ms", (unsigned long)timeout_ms);
    return ESP_ERR_TIMEOUT;
}

// ═══════════════════════════════════════════════════════════════════════════════
// NEW: Combined connect function - tries auto-connect first, then saved networks
// ═══════════════════════════════════════════════════════════════════════════════
esp_err_t wifi_creds_connect_auto(wifi_creds_handle_t* handle, uint32_t timeout_ms) {
    if (!handle || !handle->initialized) return ESP_ERR_INVALID_STATE;
    
    ESP_LOGI(TAG, "=== WiFi Connection Strategy ===");
    ESP_LOGI(TAG, "1. Try auto-connect (ESP32 remembered network)");
    ESP_LOGI(TAG, "2. Fall back to saved networks if auto-connect fails");
    
    // Strategy 1: Wait for auto-connect (ESP32 remembers from musicbox)
    esp_err_t ret = wifi_creds_wait_for_connection(timeout_ms / 2);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Auto-connect successful!");
        return ESP_OK;
    }
    
    ESP_LOGW(TAG, "Auto-connect failed, trying saved networks...");
    
    // Strategy 2: Try our saved networks
    ret = wifi_creds_load(handle);
    if (ret != ESP_OK || handle->count == 0) {
        ESP_LOGW(TAG, "No saved networks in our storage");
        // One more try with auto-connect
        return wifi_creds_wait_for_connection(timeout_ms / 2);
    }
    
    // Try each saved network
    return wifi_creds_connect_best(handle, timeout_ms / 2);
}

esp_err_t wifi_creds_load(wifi_creds_handle_t* handle) {
    if (!handle || !handle->initialized) return ESP_ERR_INVALID_STATE;
    
    nvs_handle_t nvs;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "No saved networks found in our NVS namespace");
        return ESP_OK;  // Not an error, just no saved networks
    }
    
    uint8_t count = 0;
    ret = nvs_get_u8(nvs, NVS_KEY_COUNT, &count);
    if (ret != ESP_OK || count == 0) {
        nvs_close(nvs);
        ESP_LOGW(TAG, "No network count in NVS");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Loading %d saved networks from our NVS", count);
    handle->count = 0;
    
    for (uint8_t i = 0; i < count && i < WIFI_MAX_NETWORKS; i++) {
        char key[32];
        size_t ssid_len = WIFI_SSID_MAX_LEN;
        size_t pass_len = WIFI_PASS_MAX_LEN;
        
        snprintf(key, sizeof(key), "%s%d", NVS_KEY_SSID_PREFIX, i);
        ret = nvs_get_str(nvs, key, handle->networks[handle->count].ssid, &ssid_len);
        if (ret != ESP_OK) {
            // Antes esto se descartaba sin decir nada. Si vuelve a aparecer un
            // caso de buffer corto queremos verlo en el log, no perder la red.
            ESP_LOGW(TAG, "  [%d] no pude leer %s: %s", i, key, esp_err_to_name(ret));
            continue;
        }

        snprintf(key, sizeof(key), "%s%d", NVS_KEY_PASS_PREFIX, i);
        ret = nvs_get_str(nvs, key, handle->networks[handle->count].password, &pass_len);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "  [%d] no pude leer %s: %s", i, key, esp_err_to_name(ret));
            continue;
        }

        handle->networks[handle->count].rssi = -100;

        // OJO: se indexa por handle->count, no por i. Si una red se salta, los
        // dos índices dejan de coincidir y con `i` se imprimía una posición del
        // arreglo todavía sin inicializar.
        ESP_LOGI(TAG, "  [%d] %s", i, handle->networks[handle->count].ssid);
        handle->count++;
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
    
    // Nada de ESP_ERROR_CHECK() acá: un escaneo fallido es una condición
    // esperable (radio ocupada, driver en transición), no un bug del programa.
    // ESP_ERROR_CHECK hace abort() y reinicia la caja, y esta función corre
    // justo cuando las cosas ya van mal — sería un bucle de reinicios en el
    // peor momento posible. Devolvemos el error y que decida quien llama.
    ESP_LOGI(TAG, "Starting WiFi scan...");
    esp_err_t scan_err = esp_wifi_scan_start(&scan_cfg, true);
    if (scan_err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_scan_start: %s", esp_err_to_name(scan_err));
        return scan_err;
    }

    uint16_t ap_count = WIFI_SCAN_MAX_AP;
    wifi_ap_record_t ap_list[WIFI_SCAN_MAX_AP];

    scan_err = esp_wifi_scan_get_ap_records(&ap_count, ap_list);
    if (scan_err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_scan_get_ap_records: %s", esp_err_to_name(scan_err));
        return scan_err;
    }
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
    
    // Sort by RSSI (best signal first)
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
    if (!handle || !handle->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (handle->count == 0) {
        ESP_LOGW(TAG, "No saved networks to try");
        return ESP_ERR_NOT_FOUND;
    }
    
    // Si el escaneo falla seguimos igual: todas las redes quedan en rssi=-100 y
    // más abajo entra el intento a ciegas con la primera guardada.
    esp_err_t scan_ret = wifi_creds_scan_and_match(handle);
    if (scan_ret != ESP_OK) {
        ESP_LOGW(TAG, "Escaneo fallido (%s), sigo con las guardadas a ciegas",
                 esp_err_to_name(scan_ret));
    }

    s_is_connecting = true;

    // Ninguna red guardada apareció en el escaneo: probamos igual con la
    // primera (la más reciente que guardó musicbox). Un AP con SSID oculto
    // nunca sale en el escaneo, pero sí acepta una conexión dirigida — mismo
    // criterio que el intento a ciegas del lado de musicbox.
    bool anyVisible = false;
    for (int i = 0; i < handle->count; i++) {
        if (handle->networks[i].rssi > -100) { anyVisible = true; break; }
    }
    if (!anyVisible && handle->count > 0) {
        ESP_LOGW(TAG, "Ninguna guardada visible; intento a ciegas con \"%s\"",
                 handle->networks[0].ssid);
        handle->networks[0].rssi = -99;  // lo hace elegible en el bucle
    }

    for (int i = 0; i < handle->count; i++) {
        if (handle->networks[i].rssi <= -100) continue;

        ESP_LOGI(TAG, "Connecting to %s (RSSI: %d)...",
                handle->networks[i].ssid, handle->networks[i].rssi);

        wifi_config_t wifi_config = {0};
        strncpy((char*)wifi_config.sta.ssid, handle->networks[i].ssid,
                sizeof(wifi_config.sta.ssid));
        strncpy((char*)wifi_config.sta.password, handle->networks[i].password,
                sizeof(wifi_config.sta.password));

        // Tampoco acá usamos ESP_ERROR_CHECK: esp_wifi_connect() puede fallar
        // por causas normales (config inválida, radio ocupada). Si esta red no
        // se puede intentar, pasamos a la siguiente en vez de reiniciar.
        esp_wifi_disconnect();  // puede devolver NOT_CONNECTED, es esperable

        esp_err_t cfg_err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
        if (cfg_err != ESP_OK) {
            ESP_LOGE(TAG, "set_config para \"%s\": %s",
                     handle->networks[i].ssid, esp_err_to_name(cfg_err));
            continue;
        }

        esp_err_t con_err = esp_wifi_connect();
        if (con_err != ESP_OK) {
            ESP_LOGE(TAG, "esp_wifi_connect para \"%s\": %s",
                     handle->networks[i].ssid, esp_err_to_name(con_err));
            continue;
        }

        // Re-apply throughput optimizations after config change
        esp_wifi_set_ps(WIFI_PS_NONE);
        esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40);
        
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                              CONNECTED_BIT,
                                              pdFALSE,
                                              pdFALSE,
                                              pdMS_TO_TICKS(timeout_ms / handle->count));
        
        if (bits & CONNECTED_BIT) {
            ESP_LOGI(TAG, "Connected to %s", handle->networks[i].ssid);
            s_is_connecting = false;
            return ESP_OK;
        }
        
        ESP_LOGW(TAG, "Failed to connect to %s", handle->networks[i].ssid);
    }
    
    s_is_connecting = false;
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

// Get current connection info for debugging
esp_err_t wifi_creds_get_connection_info(int8_t* rssi, uint8_t* channel) {
    wifi_ap_record_t ap_info;
    esp_err_t ret = esp_wifi_sta_get_ap_info(&ap_info);
    if (ret == ESP_OK) {
        if (rssi) *rssi = ap_info.rssi;
        if (channel) *channel = ap_info.primary;
    }
    return ret;
}

// Check if currently connected
bool wifi_creds_is_connected(void) {
    wifi_ap_record_t ap_info;
    return (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK);
}