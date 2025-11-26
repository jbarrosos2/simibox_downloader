#pragma once

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
#define TEST_WIFI_SSID "Parcela 41 2.4"
#endif

#ifndef TEST_WIFI_PASS
#define TEST_WIFI_PASS "parcela41wifi"
#endif

// Map dev mode → the knobs you were toggling
#if DOWNLOADER_DEV_MODE
  #define FORCE_DOWNLOAD      1   // seed NVS: download=1, folder=TEST_FOLDER
  #define SEED_WIFI_CREDS     1   // seed WiFi credentials in NVS
  #define STAY_IN_DOWNLOADER  0   // set to 0 to test the full cycle
  #define DEFAULT_LOG_LEVEL   ESP_LOG_DEBUG
  #define SKIP_WIFI_SCAN      1   // skip scan, just connect directly in debug mode
#else
  #define FORCE_DOWNLOAD      0
  #define SEED_WIFI_CREDS     0
  #define STAY_IN_DOWNLOADER  0
  #define DEFAULT_LOG_LEVEL   ESP_LOG_INFO
  #define SKIP_WIFI_SCAN      0
#endif