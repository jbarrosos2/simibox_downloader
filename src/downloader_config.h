#pragma once

// ═══════════════════════════════════════════════════════════════════════════════
// DOWNLOADER CONFIGURATION - EXPERIMENTAL DOUBLE-BUFFER
// ═══════════════════════════════════════════════════════════════════════════════

// Set to 1 for dev/debug mode, 0 for normal
#ifndef DOWNLOADER_DEV_MODE
#define DOWNLOADER_DEV_MODE 1  // ENABLED FOR STANDALONE TESTING
#endif

// Default test folder if dev mode
#ifndef TEST_FOLDER
#define TEST_FOLDER "mozart"
#endif

// Test WiFi credentials for standalone mode
#ifndef TEST_WIFI_SSID
#define TEST_WIFI_SSID "Parcela 41"
#endif

#ifndef TEST_WIFI_PASS
#define TEST_WIFI_PASS "parcela41wifi"
#endif

// ═══════════════════════════════════════════════════════════════════════════════
// SD CARD SETTINGS
// ═══════════════════════════════════════════════════════════════════════════════

#ifndef SD_SPI_FREQ_KHZ
#define SD_SPI_FREQ_KHZ 26000
#endif

#ifndef SD_VERBOSE_DIAGNOSTICS
#define SD_VERBOSE_DIAGNOSTICS 0
#endif

// ═══════════════════════════════════════════════════════════════════════════════
// HTTP CONNECTION SETTINGS
// ═══════════════════════════════════════════════════════════════════════════════

#ifndef HTTP_REUSE_CONNECTION
#define HTTP_REUSE_CONNECTION 1
#endif

// HTTP receive buffer size (internal to esp_http_client / mbedTLS).
#ifndef HTTP_BUFFER_SIZE
#define HTTP_BUFFER_SIZE (32 * 1024)
#endif

// ═══════════════════════════════════════════════════════════════════════════════
// TWO-PHASE DOWNLOAD: PSRAM STAGING BUFFER
// ═══════════════════════════════════════════════════════════════════════════════
//
// STRATEGY: Decouple network from SD by downloading in two clean phases:
//
//   Phase 1 (NETWORK ONLY):  HTTP/TLS → PSRAM buffer
//     No SD writes, no SPI bus contention.
//     TCP/LwIP gets full CPU attention → window stays open → max throughput.
//
//   Phase 2 (SD ONLY):  PSRAM buffer → SD card
//     No network reads. Big sequential write → SD controller loves this.
//     WiFi is idle, no ACK pressure, no contention.
//
// For a 6 MB file with 3 MB staging buffer: 2 cycles.
// Each cycle: ~3 MB download at full speed, then ~3 MB SD write at full speed.
//
// PSRAM budget (from boot log):
//   Total mapped PSRAM:    ~4.0 MB
//   After WiFi+TLS+mount:  ~4.1 MB free
//   Staging buffer:         3.0 MB (conservative, leaves 1 MB headroom)
//
// ═══════════════════════════════════════════════════════════════════════════════

#ifndef PSRAM_STAGING_BUFFER_SIZE
#define PSRAM_STAGING_BUFFER_SIZE (3 * 1024 * 1024)  // 3 MB
#endif

// SD write chunk size during Phase 2.
// We break the big PSRAM→SD write into 128 KB pieces with a 1ms yield
// between each, so FreeRTOS system tasks can run (watchdog, timers, etc.).
// The SD card doesn't care — these writes are still sequential.
#ifndef SD_WRITE_CHUNK_SIZE
#define SD_WRITE_CHUNK_SIZE (128 * 1024)
#endif

// ═══════════════════════════════════════════════════════════════════════════════
// MAP DEV MODE → DERIVED SETTINGS
// ═══════════════════════════════════════════════════════════════════════════════
#if DOWNLOADER_DEV_MODE
  #define FORCE_DOWNLOAD      1
  #define SEED_WIFI_CREDS     1
  #define STAY_IN_DOWNLOADER  0
  #define DEFAULT_LOG_LEVEL   ESP_LOG_DEBUG
  #define SKIP_WIFI_SCAN      1
#else
  #define FORCE_DOWNLOAD      0
  #define SEED_WIFI_CREDS     0
  #define STAY_IN_DOWNLOADER  0
  #define DEFAULT_LOG_LEVEL   ESP_LOG_INFO
  #define SKIP_WIFI_SCAN      0
#endif

// ═══════════════════════════════════════════════════════════════════════════════
// MEMORY NOTES (WROVER with 4 MB PSRAM) - UPDATED FOR DOUBLE-BUFFER
// ═══════════════════════════════════════════════════════════════════════════════
// 
// ESP32-WROVER heap after WiFi + TLS + SD mount:
//   Internal DRAM free:  ~99 KB   (from boot log)
//   PSRAM free:          ~4.1 MB  (from boot log)
//
// Allocation plan:
//   PSRAM staging buffer:  3.0 MB  (MALLOC_CAP_SPIRAM)
//   HTTP buffer:          32 KB    (internal or PSRAM via esp_http_client)
//   File I/O buffer:      16 KB    (static, .bss → internal DRAM)
//   Remaining PSRAM:      ~1.0 MB  (headroom for WiFi/LWIP PSRAM allocs)
//
// The key insight of two-phase download:
//   Phase 1: Only WiFi/TLS active → no SPI bus contention
//   Phase 2: Only SD active → no TCP stalls
//   Result:  Both subsystems run at their natural peak, never fighting.
//
// ═══════════════════════════════════════════════════════════════════════════════