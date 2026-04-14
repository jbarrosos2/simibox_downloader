/* simibox_boot_manager_idf.h - Pure ESP-IDF Boot Manager
 * 
 * Updated to use shared NVS protocol with musicbox firmware.
 * See simibox_nvs_protocol.h for the communication contract.
 */
#ifndef SIMIBOX_BOOT_MANAGER_IDF_H
#define SIMIBOX_BOOT_MANAGER_IDF_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "nvs.h"

// Include shared protocol definitions
#include "simibox_nvs_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

// Boot state structure - holds all persistent state
typedef struct {
    boot_reason_t reason;
    char folder[64];
    uint8_t retry_count;
    download_error_t last_error;
    uint8_t volume;
    uint8_t max_volume;
    char last_ssid[32];
    uint32_t download_size;
    uint8_t download_progress;
} boot_state_t;

// ═══════════════════════════════════════════════════════════════════════════════
// INITIALIZATION
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * Initialize the boot manager.
 * Must be called before any other boot_mgr functions.
 * Opens NVS and loads current state.
 */
esp_err_t boot_mgr_init(void);

// ═══════════════════════════════════════════════════════════════════════════════
// STATE MANAGEMENT
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * Save boot state to NVS.
 * @param state Pointer to state structure to save
 */
esp_err_t boot_mgr_save_state(const boot_state_t* state);

/**
 * Load boot state from NVS.
 * @param state Pointer to state structure to fill
 */
esp_err_t boot_mgr_load_state(boot_state_t* state);

/**
 * Clear all boot state (reset to defaults).
 */
esp_err_t boot_mgr_clear_state(void);

// ═══════════════════════════════════════════════════════════════════════════════
// DOWNLOAD WORKFLOW
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * Request a download. Called by musicbox before switching to downloader.
 * Sets reason to BOOT_REASON_DOWNLOAD_REQUEST.
 * 
 * @param folder Folder name to download (e.g., "mozart")
 * @param expected_size Expected total download size (0 if unknown)
 */
esp_err_t boot_mgr_request_download(const char* folder, uint32_t expected_size);

/**
 * Update download progress (0-100%).
 * Can be called during download to track progress.
 */
esp_err_t boot_mgr_update_progress(uint8_t percent);

/**
 * Report successful download completion.
 * Sets reason to BOOT_REASON_DOWNLOAD_SUCCESS.
 * Called by downloader before switching back to musicbox.
 */
esp_err_t boot_mgr_report_success(void);

/**
 * Report download failure.
 * Sets reason to BOOT_REASON_DOWNLOAD_FAILED and increments retry counter.
 * 
 * @param error Error code indicating what failed
 */
esp_err_t boot_mgr_report_failure(download_error_t error);

/**
 * Check if a download is needed.
 * Returns true if reason == BOOT_REASON_DOWNLOAD_REQUEST and folder is set.
 */
bool boot_mgr_needs_download(void);

/**
 * Get human-readable error string.
 */
const char* boot_mgr_get_error_string(download_error_t error);

// ═══════════════════════════════════════════════════════════════════════════════
// VOLUME SETTINGS (Persisted across reboots)
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t boot_mgr_set_volume(uint8_t vol);
esp_err_t boot_mgr_get_volume(uint8_t* vol);

#ifdef __cplusplus
}
#endif

#endif /* SIMIBOX_BOOT_MANAGER_IDF_H */
