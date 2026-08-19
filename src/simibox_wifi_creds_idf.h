/* simibox_wifi_creds_idf.h - Pure ESP-IDF WiFi Credential Management */
#ifndef SIMIBOX_WIFI_CREDS_IDF_H
#define SIMIBOX_WIFI_CREDS_IDF_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_wifi.h"
#include "nvs.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WIFI_MAX_NETWORKS 5

// Estos son tamaños de BUFFER, no longitudes máximas de contenido: incluyen el
// terminador. nvs_get_str() exige que el buffer tenga strlen+1 bytes y si no
// devuelve ESP_ERR_NVS_INVALID_LENGTH; en wifi_creds_load() eso hace `continue`
// y la red se descarta EN SILENCIO.
//
//   - SSID: máximo 32 caracteres por norma 802.11 -> 33 bytes.
//     Con 32 el caso límite (SSID de 32 caracteres) nunca se cargaba.
//   - Contraseña: passphrase WPA2 de hasta 63 caracteres, pero también se
//     acepta una PSK de 64 dígitos hexadecimales -> 65 bytes.
//
// El strncpy() posterior hacia wifi_config_t (sta.ssid[32] / sta.password[64])
// sigue siendo correcto: copia como mucho el tamaño del destino y, en el caso
// límite, lo deja sin terminador, que es justo el formato que espera ESP-IDF.
#define WIFI_SSID_MAX_LEN 33
#define WIFI_PASS_MAX_LEN 65
#define WIFI_SCAN_MAX_AP 20

typedef struct {
    char ssid[WIFI_SSID_MAX_LEN];
    char password[WIFI_PASS_MAX_LEN];
    int8_t rssi;
} wifi_network_t;

typedef struct {
    wifi_network_t networks[WIFI_MAX_NETWORKS];
    uint8_t count;
    bool initialized;
} wifi_creds_handle_t;

// Initialize WiFi subsystem
esp_err_t wifi_creds_init(wifi_creds_handle_t* handle);

// Load saved networks from our NVS namespace
esp_err_t wifi_creds_load(wifi_creds_handle_t* handle);

// Scan for APs and match against saved networks
esp_err_t wifi_creds_scan_and_match(wifi_creds_handle_t* handle);

// Connect to best saved network (scan + sort by RSSI + connect)
esp_err_t wifi_creds_connect_best(wifi_creds_handle_t* handle, uint32_t timeout_ms);

// ═══════════════════════════════════════════════════════════════════════════════
// NEW: Auto-connect functions (leverage ESP32's remembered network from musicbox)
// ═══════════════════════════════════════════════════════════════════════════════

// Wait for connection (auto-connect or already connected)
// ESP32 remembers last network and auto-reconnects on boot
esp_err_t wifi_creds_wait_for_connection(uint32_t timeout_ms);

// RECOMMENDED: Combined connect - tries auto-connect first, then saved networks
// This is the function you should use in most cases!
esp_err_t wifi_creds_connect_auto(wifi_creds_handle_t* handle, uint32_t timeout_ms);

// Check if currently connected
bool wifi_creds_is_connected(void);

// ═══════════════════════════════════════════════════════════════════════════════

// Get current connection info
esp_err_t wifi_creds_get_current_ssid(char* ssid, size_t max_len);
esp_err_t wifi_creds_get_connection_info(int8_t* rssi, uint8_t* channel);

#ifdef __cplusplus
}
#endif

#endif /* SIMIBOX_WIFI_CREDS_IDF_H */