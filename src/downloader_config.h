#pragma once

// Set to 1 for dev/debug mode, 0 for normal
#ifndef DOWNLOADER_DEV_MODE
#define DOWNLOADER_DEV_MODE 0
#endif

// Default test folder if dev mode
#ifndef TEST_FOLDER
#define TEST_FOLDER "mozart"
#endif

// Map dev mode → the knobs you were toggling
#if DOWNLOADER_DEV_MODE
  #define FORCE_DOWNLOAD      1   // seed NVS: download=1, folder=TEST_FOLDER
  #define STAY_IN_DOWNLOADER  1   // do not boot musicbox after success
  #define DEFAULT_LOG_LEVEL   ESP_LOG_DEBUG
#else
  #define FORCE_DOWNLOAD      0
  #define STAY_IN_DOWNLOADER  0
  #define DEFAULT_LOG_LEVEL   ESP_LOG_INFO
#endif
