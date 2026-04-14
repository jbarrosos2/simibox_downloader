#pragma once

// ═══════════════════════════════════════════════════════════════════════════════
// DOWNLOADER CONFIGURATION
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

// SPI frequency for SD card.
#ifndef SD_SPI_FREQ_KHZ
#define SD_SPI_FREQ_KHZ 20000
#endif

#ifndef SD_VERBOSE_DIAGNOSTICS
#define SD_VERBOSE_DIAGNOSTICS 0  // DISABLED for faster boot
#endif

// ═══════════════════════════════════════════════════════════════════════════════
// HTTP CONNECTION SETTINGS
// ═══════════════════════════════════════════════════════════════════════════════

// Reuse HTTP client across file downloads (saves TLS handshake per file).
// Each handshake costs 2-3 seconds.  With 4 files that's 8-12 seconds saved.
// WROVER has enough heap (PSRAM) to keep the TLS context alive.
#ifndef HTTP_REUSE_CONNECTION
#define HTTP_REUSE_CONNECTION 1
#endif

// HTTP receive buffer size.
// With PSRAM available, 32 KB is safe.  The TLS layer reads into this buffer
// before handing data to our download loop.
#ifndef HTTP_BUFFER_SIZE
#define HTTP_BUFFER_SIZE (32 * 1024)
#endif

// Download read buffer size — the chunk we read from HTTP and write to SD.
// With PSRAM we can go to 64 KB, halving the number of read/write cycles
// and reducing context-switch overhead between WiFi and SD tasks.
// This buffer is allocated in PSRAM (see simibox_download.cpp).
#ifndef DOWNLOAD_READ_BUFFER_SIZE
#define DOWNLOAD_READ_BUFFER_SIZE (64 * 1024)
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
// MEMORY NOTES (WROVER with 4 MB PSRAM)
// ═══════════════════════════════════════════════════════════════════════════════
// 
// ESP32-WROVER heap breakdown:
//   Internal DRAM: ~300 KB total
//     WiFi:        ~40 KB  (must be internal)
//     TLS:         ~40 KB  (mbedtls context, must be internal)
//     SD/FAT:      ~10 KB  (DMA buffers, must be internal)
//     App code:    ~20 KB
//     Free DRAM:   ~190 KB
//
//   PSRAM: ~4 MB total
//     Download buffer:  64 KB  (allocated in PSRAM)
//     HTTP buffer:      32 KB  (internal, used by esp_http_client)
//     Free PSRAM:       ~3.9 MB
//
// The key insight: WiFi/TLS/DMA buffers MUST stay in internal DRAM,
// but our download read buffer (the big one we control) can safely
// go to PSRAM since we just memcpy from it to fwrite().
//
// ═══════════════════════════════════════════════════════════════════════════════